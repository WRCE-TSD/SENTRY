# SENTRY NERF Turret Project

Welcome to the repository for the **SENTRY NERF Turret**, an advanced and interactive project designed for educational purposes, robotics experimentation, and practical learning in robotics and control engineering.

---

## Project Overview

This project features a fully automated NERF turret with pan and tilt capabilities, flywheel-based ball launching, and integrated sensors for precise control and monitoring. It leverages the power of MicroPython on a Raspberry Pi Pico and includes an array of sensors and actuators for a complete, interactive robotic experience.

---

## Key Features

- **Automated Targeting and Firing:** Integrated motors for pan, tilt, and flywheel-based ball launching.
- **Sensors and Feedback:** INA260 current sensor for ball detection, BNO055 IMU for orientation, and QRD1114 optical sensors for accurate ball detection and system monitoring.
- **Interactive Controls:** Push-button firing, visual feedback via WS2812B RGB LEDs, and laser pointer activation.
- **Expandable System:** UART communication ready for integration with Raspberry Pi 4B and Oak-D Lite camera for advanced computer vision applications.

---

## Hardware Components

- **Pan/Tilt Head:** [Proaim Senior 2-axis Pan/Tilt Head](https://www.bhphotovideo.com/c/product/1555779-REG/)
- **Controller:** Raspberry Pi Pico running MicroPython
- **Motors:** MOSFET-driven motors for the flywheel and feed mechanism, BD62120AEFJ-E2 motor drivers for pan and tilt.
- **Sensors:**
  - INA260 Current Sensor
  - BNO055 IMU
  - QRD1114 Optical Sensors
- **Indicators and Outputs:**
  - WS2812B RGB LEDs
  - Laser pointer control
  - Button interface for manual firing

# Version 2.3
The current release on this site is 2.3.

Working on version 3 which will include among other things:
- Various fixes discovered after running these for one semester
- Encoder integration on feed motor
- ToF sensor for distance measuring
- QRD1114's moved to flywheels for speed measurement
- New optical sensors for counting/launch speed
- More robust I2C integration
- Better documentation/BOM

**this version is released here as a beta**

Almost everything is done through Autodesk Fusion, which doesn't natively port to GitHub, so updates are not published regularly.
Edit 3/28/25: I made a [script to push Fusion F3D files to Github](https://github.com/zcohenld/FusionToGitHub) so I'll be pushing the newest version, which is untested until I do a release, to this GitHub.


---
# Contribution

Contributions and improvements are welcome! Feel free to submit pull requests or open issues for discussions and enhancements.
