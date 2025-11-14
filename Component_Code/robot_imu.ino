#include <Arduino.h>
#include <Wire.h>
#include <LSM6.h>

// Create an instance of the LSM6 library
LSM6 imu;

// --- Global Variables for IMU ---
float currentAngle = 0.0;      // The robot's current angle in degrees, calculated from the gyro
float gyroZ_offset = 0.0;      // Gyroscope's Z-axis offset for calibration
unsigned long lastIMUReadTime = 0; // To calculate the time difference (deltaTime) for integration

/**
 * @brief Reads the gyroscope and calculates the current angle by integration.
 * This function should be called continuously in the main loop.
 */
void readIMU() {
    // Read the raw sensor data from the IMU
    imu.read();

    // Get the current time in microseconds for precise calculation

    unsigned long currentTime = micros();
    
    // Skip the first loop to get a valid deltaTime and avoid a large initial jump
    if (lastIMUReadTime == 0) {
        lastIMUReadTime = currentTime;
        return;
    }

    // Calculate the time difference since the last reading, in seconds
    float deltaTime = (currentTime - lastIMUReadTime) / 1000000.0; 
    lastIMUReadTime = currentTime; // Update the time for the next loop

    // Get the angular velocity around Z-axis (in degrees per second) and correct for drift
    float gyroZ_dps = imu.calcGyro(imu.gz) - gyroZ_offset;

    // Integrate the angular velocity to get the angle change: Angle = Speed * Time
    currentAngle += gyroZ_dps * deltaTime;
}

/**
 * @brief Calibrates the gyroscope by reading a number of samples while the robot is stationary.
 * This calculates the average drift (offset) which is then subtracted in readIMU().
 */
void calibrateIMU() {
    Serial.println("Calibrating IMU... Keep the robot perfectly still for a few seconds.");
    float totalGyroZ = 0;
    const int samples = 500; // Number of samples to average

    for (int i = 0; i < samples; i++) {
        imu.read();
        // Convert raw gyro reading to degrees per second and add to total
        totalGyroZ += imu.calcGyro(imu.gz);
        delay(3); // Small delay between readings
    }

    // Calculate the average offset
    gyroZ_offset = totalGyroZ / samples;

    Serial.print("Calibration complete. Gyro Z offset (dps): ");
    Serial.println(gyroZ_offset);
}

// --- Setup Function ---
void setup() {
  // Start serial communication for debugging
  Serial.begin(9600); 
  Serial.println("IMU Test Program Starting...");

  // Initialize I2C communication
  Wire.begin();

  // Initialize the IMU sensor
  if (!imu.init()) {
      Serial.println("Failed to detect and initialize IMU!");
      // Halt execution if the IMU is not found
      while (1); 
  }
  imu.enableDefault();
  Serial.println("IMU initialized successfully.");

  // Calibrate the gyroscope. It's crucial for the robot to be stationary during this process.
  calibrateIMU();
}

// --- Main Loop for Testing ---
void loop() {
  // Continuously update the angle from the IMU
  readIMU();

  // Print the current angle to the Serial Monitor for testing
  Serial.print("Current Angle (Z-axis): ");
  Serial.println(currentAngle);

  // Add a small delay to make the output readable
  delay(100);
}