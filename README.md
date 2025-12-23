# self-balancing-robot

## 📌 Overview
This project will give you a overview of implementation of a self balacing robot with the code, schematics and the components used in the project.

This won't be a complete step by step just a rough overview with code.

## 🎯 Steps

1. Start with basic motor and sensor control.
2. Make a frame for the self balancing robot.
3. Integrate both motor, motor driver, and imu to get real time data.
4. Install a PID control in the robot to make it fully functional.

## 🛠️ Hardware

1. Microcontroller: ESP32
2. Battery : 2x 18650 3700Mah
3. Buck Converter: LM2596
4. IMU: MPU6050
5. Motors: 2x N20 6V 600RPM

## 💻 Software And Tools

1. Arduino IDE/ Toolchain
2. Libraries: MPU6050.h

## ⚙️ Working Principle

1. The IMU sensor measures the robot’s tilt angle and angular velocity.
2. Sensor data is processed to estimate the current orientation.
3. PID calculates the error between the desired angle and the measured angle.
4. Based on the PID output, PWM signals are generated to control motor speed and direction.
5. This process runs continuously in real time to maintain balance. 

## 🧠 Control Algorithm (PID)

The PID controller consists of:
1. Proportional (P): Corrects current error  
2. Integral (I): Eliminates steady-state error  
3. Derivative (D): Predicts future error and improves stability





