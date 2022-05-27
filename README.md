# Kalman_MPU6050
This code repository collects, process, and visualizes accelerometer and gyroscope data from an MPU6050 sensor using a Kalman Filter on a Raspberry Pi.

# Code Dependencies
This repository requires the smbus, mpu6050, numpy, matplotlib libraries. Below is the code to install these libraries.
```bash
sudo apt install python3-smbus
```
```bash
pip install mpu6050-raspberrypi
```
```bash
pip install -U matplotlib
```
```bash
pip install -U numpy
```

# Hardware
This code should work for the Rapberry Pi 3B+ and 4B models. 

First, make sure to enable your Raspberry Pi's I2C interface using the Raspberry Pi Configuration panel. 

In order to collect data, the MPU6050 sensor is required. To connect the MPU6050 to the Raspberry Pi, connect its VCC pin to the VCC pin of the RPI. The GND pin to the GND pin of the RPI. The SCL pin to the SCL pin of the RPI (Pin5). The SDA pin to the SDA pin of the RPI (Pin3). The picture below is a circuit diagram of the application.


<img src="Circuit.jpg" width="1000">

# Usage
To collect, process, and visualize accelerometer data from the MPU6050 sensor, type in the following code:
```python
python accelerometer.py
```

To collect, process, and visualize gyroscope data from the MPU6050 sensor, type in the following code:
```python
python gyro.py
```
