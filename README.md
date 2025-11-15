# Team Manatee — Robot Firmware & Unit Tests

Small collection of Arduino sketches for the SYSC4805 Team Manatee Autonomous Snow Plow.

Quick start
- Requirements: Arduino IDE or PlatformIO, Arduino Due (3.3V), libraries: LSM6/LIS3MDL, VL53L1X.
- To run: open a .ino (Component_Code/ or Unit_Test/), upload to the board, open Serial Monitor (match baud).

Key files
- Component_Code/robot_main.ino — main orchestrator
- Component_Code/robot_motor.ino — motor control API
- Component_Code/robot_imu.ino — IMU integration & calibration
- Component_Code/robot_border.ino — line sensor helpers
- Unit_Test/* — individual hardware tests (IMU, motors, encoders, TOF, ultrasonic, IR)

Running unit tests
- Upload one sketch from Unit_Test/ at a time.
- Watch serial output for PASS/FAIL and diagnostic prints.
