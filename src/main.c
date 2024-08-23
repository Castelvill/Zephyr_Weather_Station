//Authors: Mateusz Matkowski (145432) and Mikołaj Starzak (158958)

#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#define GATT_PERM_READ_MASK     (BT_GATT_PERM_READ | \
				 BT_GATT_PERM_READ_ENCRYPT | \
				 BT_GATT_PERM_READ_AUTHEN)
#define GATT_PERM_WRITE_MASK    (BT_GATT_PERM_WRITE | \
				 BT_GATT_PERM_WRITE_ENCRYPT | \
				 BT_GATT_PERM_WRITE_AUTHEN)

#ifndef CONFIG_BT_TEMPS_DEFAULT_PERM_RW_AUTHEN
#define CONFIG_BT_TEMPS_DEFAULT_PERM_RW_AUTHEN 0
#endif
#ifndef CONFIG_BT_TEMPS_DEFAULT_PERM_RW_ENCRYPT
#define CONFIG_BT_TEMPS_DEFAULT_PERM_RW_ENCRYPT 0
#endif

#ifndef CONFIG_BT_PRESSS_DEFAULT_PERM_RW_AUTHEN
#define CONFIG_BT_PRESSS_DEFAULT_PERM_RW_AUTHEN 0
#endif
#ifndef CONFIG_BT_PRESSS_DEFAULT_PERM_RW_ENCRYPT
#define CONFIG_BT_PRESSS_DEFAULT_PERM_RW_ENCRYPT 0
#endif

#define TEMPS_GATT_PERM_DEFAULT (						\
	CONFIG_BT_TEMPS_DEFAULT_PERM_RW_AUTHEN ?				\
	(BT_GATT_PERM_READ_AUTHEN | BT_GATT_PERM_WRITE_AUTHEN) :	\
	CONFIG_BT_TEMPS_DEFAULT_PERM_RW_ENCRYPT ?				\
	(BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT) :	\
	(BT_GATT_PERM_READ | BT_GATT_PERM_WRITE))			\

#define PRESSS_GATT_PERM_DEFAULT (						\
	CONFIG_BT_PRESSS_DEFAULT_PERM_RW_AUTHEN ?				\
	(BT_GATT_PERM_READ_AUTHEN | BT_GATT_PERM_WRITE_AUTHEN) :	\
	CONFIG_BT_PRESSS_DEFAULT_PERM_RW_ENCRYPT ?				\
	(BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT) :	\
	(BT_GATT_PERM_READ | BT_GATT_PERM_WRITE))		

static bool tempf_ntf_enabled;
static bool pressf_ntf_enabled;

//Create an advertisement for the service
static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(BT_UUID_ESS_VAL))
};


//Manage the system for handling temperature notification changes in a Bluetooth application
static void temps_ntf_changed(bool enabled){
	tempf_ntf_enabled = enabled;
	printk("Temp notification status changed: %s\n", enabled ? "enabled" : "disabled");
}
struct bt_temp_cb {
	void (*ntf_changed)(bool enabled);
	/** Internal member to form a list of callbacks */
	sys_snode_t _node;
};
static struct bt_temp_cb temp_cb = {
	.ntf_changed = temps_ntf_changed,
};
static sys_slist_t temp_cbs = SYS_SLIST_STATIC_INIT(&temp_cbs);
static void tempmc_ccc_cfg_changed(const struct bt_gatt_attr *attr, int16_t value){
	ARG_UNUSED(attr);
	struct bt_temp_cb *listener;
	bool notif_enabled = (value == BT_GATT_CCC_NOTIFY);
	SYS_SLIST_FOR_EACH_CONTAINER(&temp_cbs, listener, _node) {
		if (listener->ntf_changed) {
			listener->ntf_changed(notif_enabled);
		}
	}
}

//Manage the system for handling notification about pressure changes in a Bluetooth application
static void presss_ntf_changed(bool enabled){
	pressf_ntf_enabled = enabled;
	printk("Press notification status changed: %s\n", enabled ? "enabled" : "disabled");
}
struct bt_press_cb {
	void (*pntf_changed)(bool enabled);
	/** Internal member to form a list of callbacks */
	sys_snode_t _node;
};
static struct bt_press_cb press_cb = {
	.pntf_changed = presss_ntf_changed,
};
static sys_slist_t press_cbs = SYS_SLIST_STATIC_INIT(&press_cbs);
static void pressmc_ccc_cfg_changed(const struct bt_gatt_attr *attr, int16_t value){
	ARG_UNUSED(attr);

	struct bt_press_cb *listener;

	bool notif_enabled = (value == BT_GATT_CCC_NOTIFY);

	SYS_SLIST_FOR_EACH_CONTAINER(&press_cbs, listener, _node) {
		if (listener->pntf_changed) {
			listener->pntf_changed(notif_enabled);
		}
	}
}


//Define Environmental Sensing Service
BT_GATT_SERVICE_DEFINE(ess_srv,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_ESS),
	BT_GATT_CHARACTERISTIC(BT_UUID_TEMPERATURE, BT_GATT_CHRC_NOTIFY, NULL, NULL, NULL, NULL),
	BT_GATT_CCC(tempmc_ccc_cfg_changed, TEMPS_GATT_PERM_DEFAULT),
	BT_GATT_CHARACTERISTIC(BT_UUID_PRESSURE, BT_GATT_CHRC_NOTIFY, NULL, NULL, NULL, NULL),
	BT_GATT_CCC(pressmc_ccc_cfg_changed, PRESSS_GATT_PERM_DEFAULT),
);

static void connected(struct bt_conn *conn, uint8_t err){
	if (err) {
		printk("Connection failed (err 0x%02x)\n", err);
	} else {
		printk("Connected\n");
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason){
	printk("Disconnected (reason 0x%02x)\n", reason);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};


static void bt_ready(void){
	int err;
	printk("Bluetooth initialized\n");
	//Start the service advertisement
	err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return;
	}
	printk("Advertising successfully started\n");
}

static void auth_cancel(struct bt_conn *conn){
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Pairing cancelled: %s\n", addr);
}

static struct bt_conn_auth_cb auth_cb_display = {
	.cancel = auth_cancel,
};


//Register temperature service callback.
int bt_temp_cb_register(struct bt_temp_cb *cb){
	if(cb == NULL) {
		return -EINVAL;
	}
	sys_slist_append(&temp_cbs, &cb->_node);
	return 0;
}
//Notify the service about the new temperature value.
int bt_temps_notify(uint16_t temperature){
	int rc = bt_gatt_notify(NULL, &ess_srv.attrs[1], &temperature, sizeof(temperature));
	return rc == -ENOTCONN ? 0 : rc;
}
static void temp_notify(struct sensor_value temp){
	double tempd = sensor_value_to_double(&temp);
	uint16_t temperature = tempd * 100;

	if (tempf_ntf_enabled) {
		bt_temps_notify(temperature);
	}
}

//Register pressure service callback.
int bt_press_cb_register(struct bt_press_cb *cb){
	if(cb == NULL) {
		return -EINVAL;
	}
	sys_slist_append(&press_cbs, &cb->_node);
	return 0;
}
//Notify the service about the new pressure value.
int bt_presss_notify(uint32_t pressure){
	int rc = bt_gatt_notify(NULL, &ess_srv.attrs[4], &pressure, sizeof(pressure));
	return rc == -ENOTCONN ? 0 : rc;
}
static void press_notify(struct sensor_value press){
	double pressured = sensor_value_to_double(&press);
	uint32_t pressure = pressured * 10000;

	if (pressf_ntf_enabled) {
		bt_presss_notify(pressure);
	}
}


//Connect BME280/BMP280 sensor to the application (BMP280 is compatible with BME280's code for temperature and pressure readings).
static const struct device *get_bme280_device(void){
	const struct device *const dev = DEVICE_DT_GET_ANY(bosch_bme280);
	if (dev == NULL) {
		/* No such node, or the node does not have status "okay". */
		printk("\nError: no device found.\n");
		return NULL;
	}
	if (!device_is_ready(dev)) {
		printk("\nError: Device \"%s\" is not ready; "
		       "check the driver initialization logs for errors.\n",
		       dev->name);
		return NULL;
	}
	printk("Found device \"%s\", getting sensor data\n", dev->name);
	return dev;
}

int main(void){
	int err;

	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

	bt_ready();
	bt_conn_auth_cb_register(&auth_cb_display);

	bt_temp_cb_register(&temp_cb);
	bt_press_cb_register(&press_cb);

	const struct device *dev = get_bme280_device();
	if (dev == NULL) {
		return 0;
	}
	struct sensor_value temp, press, humidity;

	while (1) {
		//Measure the current temperature and pressure.
		sensor_sample_fetch(dev);
		sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);
		sensor_channel_get(dev, SENSOR_CHAN_PRESS, &press);
		sensor_channel_get(dev, SENSOR_CHAN_HUMIDITY, &humidity);

		printk("temp: %d.%06d; press: %d.%06d;\n",
		      temp.val1, temp.val2, press.val1, press.val2);

		//Notify the service about the temperature and pressure changes.
		temp_notify(temp);
		press_notify(press);

		k_sleep(K_SECONDS(5));
	}
	return 0;
}
