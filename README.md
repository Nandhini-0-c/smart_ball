# 🏏 Ball IQ – Revolutionizing Play with Smart Ball

An internally funded student project focused on enhancing cricket training using an embedded smart ball that tracks motion and spin in real-time.

---

## 📌 Overview

This project introduces **Ball IQ**, a smart cricket ball system equipped with an **MPU6050 IMU sensor** and **ESP32 Pico**, capable of measuring:

- 🌀 **Spin rate**
- 🏃‍♂️ **Throw velocity**
- 📐 **Roll & Pitch**
- 📊 **Acceleration and Gyro (Z-axis)**

All data is streamed wirelessly via **Serial Bluetooth Terminal**, enabling real-time performance feedback for players and coaches.

---

## ⚙️ Hardware Used

| Component              | Description                           |
|------------------------|---------------------------------------|
| ESP32 Pico             | Main controller with Bluetooth        |
| MPU6050                | 6-axis accelerometer + gyroscope      |
| 3.7V LiPo Battery      | Power source                          |
| TP4056 Charging Module | Battery charging and protection       |
| Buck-Boost Converter   | Regulates voltage to ESP32/MPU6050    |
| Cricket Ball Shell     | Embeds all electronics inside         |

---

## 🧠 Features

- **Motion Tracking:** Captures linear acceleration and angular velocity.
- **Spin Rate Estimation:** Converts gyro Z data to RPM.
- **Velocity Calculation:** Uses Kalman filtering and acceleration integration.
- **Orientation Detection:** Complementary filter gives pitch and roll angles.
- **Release Detection:** Detects when the ball is thrown based on acceleration drop.
- **Bluetooth Integration:** Data streamed live to a mobile device.

---

## 📲 Mobile App

We use the **Serial Bluetooth Terminal** app (MIT licensed, open-source) available on Android to display:



  
 
 
 
 
