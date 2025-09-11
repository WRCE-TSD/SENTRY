# SENTRY NERF Turret Project

Welcome to the repository for the **SENTRY NERF Turret**, an advanced educational mechatronics platform developed for robotics experimentation, controls engineering, and hands-on learning.  

This project demonstrates how mechanical, electrical, and software systems come together in a real, fully functional robotic turret. It provides students and researchers with an accessible platform to explore **feedback control, sensor fusion, actuation, and system integration** — using NERF balls as the medium for experimentation.

![SENTRY Turret](https://github.com/zcohen-nerd/SENTRY/blob/main/1000007922.jpg "SENTRY Turret")

---

## Project Overview

The SENTRY turret is a fully automated NERF blaster mounted on a 2-axis pan/tilt head, controlled by a Raspberry Pi Pico running MicroPython.  

The system integrates **sensors, encoders, and current monitoring** to deliver closed-loop control, ball detection, and real-time feedback. A custom breakout board centralizes the I²C architecture and provides additional ADC channels, making SENTRY a modular, expandable, and classroom-ready platform.

![Isometric View](https://github.com/zcohenld/SENTRY/blob/main/SENTRY%20Iso.png "Isometric View of SENTRY Gun")  
![Cross Section](https://github.com/zcohenld/SENTRY/blob/main/SENTRY%20Cross%20Section.png "Section View of SENTRY Gun")

---

## Key Features

- **Automated Targeting and Firing**
  - Dual flywheels for ball launching
  - AS5600 encoder-driven feed belt for reliable ball delivery
  - INA260 current-spike detection for ball counting

- **Feedback & Control**
  - BNO055 IMU for orientation and automatic tilt leveling
  - VL53L4CX Time-of-Flight (ToF) sensor for range measurement
  - QRD1114 optical sensors relocated to flywheels for rotational speed measurement
  - TCND5000 optical sensors to detect ball launch and calculate speed of ball
  - INA260 current monitoring for real-time performance feedback
  - ADS1115 I²C ADC breakout for expanded analog inputs

- **Interactive System**
  - Push-button control for manual firing
  - WS2812B RGB LEDs for real-time visual status and diagnostics

- **Modular & Expandable**
  - Unified I²C bus for all sensors, routed through a custom breakout board
  - UART communication ready for integration with a Raspberry Pi 4B and Oak-D Lite camera for advanced vision-based tracking
  - Designed for educational labs, adaptable for both controls and robotics research

---

## Hardware Components

- **Mechanical Base**
  - [Proaim Senior 2-axis Pan/Tilt Head](https://www.bhphotovideo.com/c/product/1555779-REG/)

- **Controller**
  - Raspberry Pi Pico running MicroPython  
  - Custom breakout board consolidating I²C and ADC expansion

- **Motors**
  - Flywheel motor (MOSFET-controlled)  
  - Feed belt motor with quadrature encoder  
  - Pan & tilt motors driven by BD62120AEFJ-E2 H-bridge drivers

- **Sensors**
  - **INA260** — current sensor for ball count & system load  
  - **BNO055** — IMU for orientation and tilt leveling  
  - **VL53L4CX** — ToF distance sensor for range measurement & targeting feedback  
  - **QRD1114** — optical sensors for flywheel speed measurement  
  - **ADS1115** — I²C ADC for additional analog input channels

- **Indicators & Controls**
  - Push button (manual firing trigger)  
  - WS2812B RGB LEDs for status indication  

---

## Version 3 Highlights

Version 3 represents the most robust and educationally capable version of SENTRY to date, with major system-level improvements:

- Unified **I²C bus architecture** with custom breakout board  
- **Encoder integration** on feed belt motor  
- **VL53L4CX ToF rangefinding** for advanced control and targeting  
- **Relocated QRD1114s** for direct flywheel speed measurement  
- More accurate **ball detection via INA260 current spikes**  
- **ADS1115 I²C ADC** for expanded analog sensor support  
- Integrated **self-test routines** for INA260, BNO055, and QRD1114 sensors  
- **Improved documentation and BOM** for replication and classroom use  

---

## Educational Applications

SENTRY is more than a turret — it is a **teaching and research platform**. It enables:

- Demonstration of **closed-loop feedback control**  
- Practical labs on **PID tuning** and system stability  
- Exploration of **sensor fusion and redundancy**  
- Robotics coursework involving **computer vision**, **control systems**, and **mechatronics integration**  
- A fun, tangible way to connect engineering theory to practice  

---

## To-Do List

- [ ] Generate and publish full Bill of Materials (BoM)  
- [ ] Finish development of handheld controller  
- [ ] Create system wiring/block diagram for documentation  
- [ ] Develop example student labs (PID tuning, tilt leveling, ball counting)  
- [ ] Expand Oak-D Lite + Raspberry Pi 4 integration for vision-based targeting  
- [ ] Continue refining firing logic and feed control with encoder feedback  

---

## Contribution

Contributions and improvements are welcome!  
Feel free to fork, submit pull requests, or open issues for discussion.  

---

⚡ This project is actively used in robotics and controls engineering education. Expect frequent updates as new features, documentation, and hardware iterations are released.
