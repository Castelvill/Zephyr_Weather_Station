# Zephyr and Arduino - Weather Station

Weather station app based on Zephyr Real-Time Operating System (RTOS) and Arduino Nano 33 BLE prototyping board.

## Achieved Goals

- Measure temperature and pressure with sensors connected to Arduino Nano 33 BLE.
- Use Zephyr system to send data from sensors through the Bluetooth advertisement.
- Read temerature and pressure measurements in nRFconnect android app.

## Build Zephyr

    west build -p always -b arduino nano 33 ble <path-to-zephyr-root> -DDTC OVERLAY FILE=”arduino i2c.overlay”

## Flash to Arduino Nano 33 BLE

    west flash –bossac=<path-to-bossac> –bossac-port=”COM3”

## Connection Scheme

<img src="images/scheme.png" width="400"/>

## Arduino Nano 33 BLE with sensors

<img src="images/device.jpg" width="400"/>

## nRFconnect Screenshots

<img src="images/nRFconnect_1.jpg" width="400"/>

<br>

<img src="images/nRFconnect_2.jpg" width="400"/>