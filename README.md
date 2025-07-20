Project Title:
Ball IQ – Revolutionizing Play with Smart Ball

Project Description:
This is an internally funded student project aimed at enhancing cricket training by developing a smart cricket ball that captures key motion parameters like spin, speed, acceleration, and orientation. The system uses an MPU6050 sensor and ESP32 Pico microcontroller, all embedded inside a cricket ball. Data is transmitted wirelessly using Bluetooth and displayed on the Serial Bluetooth Terminal mobile app.

Objective:
The main goal is to track spin rate, speed, pitch, roll, and impact-related data during a throw. These metrics can help players understand and improve their performance.

Hardware Used:

ESP32 Pico (microcontroller with Bluetooth)

MPU6050 (accelerometer + gyroscope)

3.7V LiPo rechargeable battery

TP4056 charging module

Buck-boost converter for power regulation

Enclosure inside a cricket ball

How It Works:
The MPU6050 captures acceleration and angular velocity data. This is processed by the ESP32, which calculates parameters like spin rate (from gyro Z-axis), speed (using acceleration data), and orientation (pitch and roll). The data is sent to a smartphone via Bluetooth and displayed in real-time using the Serial Bluetooth Terminal app.

Mobile App Interface:
We use the open-source Serial Bluetooth Terminal app (MIT License) on Android. It displays all live readings such as:

Speed in m/s

Spin rate in RPM

Roll and pitch angles

Acceleration and gyro values

Project Outcomes:
We successfully tested the smart ball by throwing it under various conditions. The system provided accurate and real-time data, helping us analyze throws in terms of speed, spin, and movement. This information can be very useful for training, coaching, and performance analysis in cricket.


  
 
    
