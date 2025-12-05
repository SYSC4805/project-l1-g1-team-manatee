# Team Manatee — Robot Firmware & Unit Tests

This repository contains Arduino sketches and unit tests for the SYSC4805 Team Manatee Autonomous Snow Plow.

## Quick Start

- **Requirements:**  
  - Arduino IDE or PlatformIO  
  - Arduino Due (3.3V)  
  - Libraries: LSM6/LIS3MDL, VL53L1X

- **To run:**  
  - Open a `.ino` file from `Component_Code/` or `Unit_Test/` directory at the repository root  
  - Upload to the board  
  - Open Serial Monitor (ensure the baud rate matches the sketch)

## Key Files & Directories

- `Component_Code/robot_main.ino` — main orchestrator
- `Component_Code/robot_motor.ino` — motor control API
- `Component_Code/robot_imu.ino` — IMU integration & calibration
- `Component_Code/robot_border.ino` — line sensor helpers
- `Unit_Test/` — individual hardware test sketches (IMU, motors, encoders, TOF, ultrasonic, IR)

## Running Unit Tests

- Upload a sketch from the `Unit_Test/` directory (one at a time)
- Watch serial output for PASS/FAIL and diagnostic messages
