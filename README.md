# Automated Car Window Control – Raspberry Pi

This project simulates the control of a car window using a DC motor, driven by a Pololu 2961 motor driver. A proximity sensor ensures user safety, and an I2C LCD display shows real-time information about the system's state. The entire system is controlled using a Raspberry Pi.

## Components Used
- Raspberry Pi 3B+
- DC Motor
- Pololu 2961 Motor Driver
- 3-Position Switch
- Proximity Sensor
- 20x4 I2C LCD Display

## Features
- **DC Motor Control**: Motor direction (open/close) and speed control through PWM signals.
- **Proximity Safety System**: Detects obstacles during window closure and automatically reverses motor direction.
- **User Interface**: Real-time display of motor direction (left/right) on an LCD connected via I2C.
- **Signal Testing**: PWM signals tested and verified using an oscilloscope.
- **Hardware Debugging**: External 9V power supply integration to ensure proper motor startup.

## Wiring Overview
- **PWM Signal**: GPIO pin 32 (PWM control to motor driver)
- **Direction Control**: GPIO pin 11
- **Switch Input**: GPIO pins 13 (Right) and 15 (Left)
- **Proximity Sensor Input**: GPIO pin 18
- **LCD I2C**: SDA (GPIO2) and SCL (GPIO3)

## How to Run the Project
1. Connect the components according to the wiring diagram.
2. Boot up the Raspberry Pi and install the required libraries:
sudo apt-get install python3-rpi.gpio python3-smbus

## Diagrams


