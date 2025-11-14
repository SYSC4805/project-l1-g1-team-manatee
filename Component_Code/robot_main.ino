#include <Arduino.h>
#include <Wire.h>
#include <LSM6.h>
LSM6 imu;

// Pin for imu, motors and encoders
const int L_F_DIR = 1; // Left Front Motor Direction Pin
const int L_F_PWM = 2; // Left Front Motor PWM Pin

const int L_R_DIR = 3; // Left Rear Motor Direction Pin
const int L_R_PWM = 4; // Left Rear Motor PWM Pin

const int R_F_DIR = 5; // Right Front Motor Direction Pin
const int R_F_PWM = 7; // Right Front Motor PWM Pin

const int R_R_DIR = 6; // Right Rear Motor Direction Pin
const int R_R_PWM = 8; // Right Rear Motor PWM Pin


// Pin for Line Follower Sensor
const int LINE_SENSOR_LEFT = A0; 
const int LINE_SENSOR_CENTER = A1; 
const int LINE_SENSOR_RIGHT = A2; 
// Parameters for line sensor 
int leftValue = 0;
int centerValue = 0;
int rightValue = 0;
int borderTriggered = 0; // Flag to indicate border detection, 1-left, 2-right, 3-center

// Pin for Ultrasonic Sensor and Infrared Sensor


// Pin for IMU (I2C)
// On most Arduino boards (Uno/Nano) SDA = , SCL = 
//const int IMU_SDA = ;
//const int IMU_SCL = ;


// Declare the variables
const float MAX_SPEED = 30.0; // cm/s
const int MAX_PWM = 255; // Adjust the PWM value if the actual max speed
const int BORDER_THRESHOLD = 500; 

// IMU Variables
float currentAngle = 0.0;      // The robot's current angle in degrees
float gyroZ_offset = 0.0;      // Gyroscope's Z-axis offset for calibration
unsigned long lastIMUReadTime = 0;

// Robot States
enum RobotState {
    IDLE,
    PLOWING_SNOW, // Plow the snow cubes when snow cubes are detected
    MOVING_AROUND, // Move around in the arena
    AVOIDING_BORDER,
  
};
RobotState currentState = IDLE;




// Motor Control Functions
void setMotorSpeed(int dirPin, int pwmPin, float speed){
  // control the motor speed
  // speed below 30 cm/s
  if (speed > MAX_SPEED) {
    speed = MAX_SPEED;
  }
  if (speed < -MAX_SPEED) {
    speed = -MAX_SPEED;
  }

  if (speed > 0) {
    digitalWrite(dirPin, HIGH); // One direction
  } else {
    digitalWrite(dirPin, LOW); // The other direction
  }
  
  int pwmValue = map(abs(speed), 0, MAX_SPEED, 0, MAX_PWM);
  analogWrite(pwmPin, pwmValue);
}

void setLeftMotor(float speed){
    setMotorSpeed(L_F_DIR, L_F_PWM, speed);
    setMotorSpeed(L_R_DIR, L_R_PWM, speed);

}

void setRightMotor(float speed){
    setMotorSpeed(R_F_DIR, R_F_PWM, speed);
    setMotorSpeed(R_R_DIR, R_R_PWM, speed);
}

void moveForward(){
  // Move the robot forward at a specified speed
    setLeftMotor(MAX_SPEED);
    setRightMotor(MAX_SPEED);
}

void moveBackward(){
  // Move the robot backward at a specified speed
    setLeftMotor(-MAX_SPEED);
    setRightMotor(-MAX_SPEED);
}

void turnLeft(){
  // Turn the robot left
    setLeftMotor(-MAX_SPEED/2);
    setRightMotor(MAX_SPEED/2);
}

void turnRight(){
  // Turn the robot right
    setLeftMotor(MAX_SPEED/2);  
    setRightMotor(-MAX_SPEED/2);
}

void stopMotors(){
  // Stop all motors
    setLeftMotor(0);
    setRightMotor(0);
} 

void moveAround(){
    // Move the robot around in the arena
    moveForward();
}

// Line Sensor Functions
void readLineSensors(){
    // Read the line follower sensors
    leftValue = analogRead(LINE_SENSOR_LEFT);
    centerValue = analogRead(LINE_SENSOR_CENTER);
    rightValue = analogRead(LINE_SENSOR_RIGHT);
    // Process the sensor values as needed
}

//IMU Functions
void readIMU(){
    // Read the IMU sensor values
    imu.read();

    // Get the current time in microseconds 
    unsigned long currentTime = micros();
    
    // Skip the first loop to get a valid deltaTime and avoid a large initial jump
    if (lastIMUReadTime == 0) {
        lastIMUReadTime = currentTime;
        return;
    }

    // Calculate time difference in seconds
    float deltaTime = (currentTime - lastIMUReadTime) / 1000000.0; 
    lastIMUReadTime = currentTime; // Update the time for the next loop

    // Get the angular velocity around Z-axis (in degrees per second) and correct for drift
    float gyroZ_dps = imu.calcGyro(imu.gz) - gyroZ_offset;

    // Integrate the angular velocity to get the angle change
    currentAngle += gyroZ_dps * deltaTime;
}

void calibrateIMU() {
    Serial.println("Calibrating IMU... Keep the robot still.");
    float totalGyroZ = 0;
    const int samples = 500;
    for (int i = 0; i < samples; i++) {
        imu.read();
        totalGyroZ += imu.calcGyro(imu.gz);
        delay(3);
    }
    gyroZ_offset = totalGyroZ / samples;
    Serial.print("Calibration complete. Gyro Z offset (dps): ");
    Serial.println(gyroZ_offset);
}





// Ultrasonic and Infrared Sensor Functions
void readUltrasonicSensor(){
    // Read the ultrasonic sensor value
}
void readInfraredSensor(){
    // Read the infrared sensor value
}



// Border Checking and Avoidance
void checkBorders(){
    // Check if the robot is close to the borders using line follower sensors
    if (currentState != MOVING_AROUND) {
        return; // Only check for borders when moving normally
    }
    borderTriggered = 0; // Reset border trigger flag

    if (leftValue > BORDER_THRESHOLD) {
        // Left border detected
        borderTriggered = 1;
        

    } else if (rightValue > BORDER_THRESHOLD) {
        // Right border detected
        borderTriggered = 2;

    } else if (centerValue > BORDER_THRESHOLD) {
        // Center border detected
        borderTriggered = 3;
    }
    
    if (borderTriggered != 0) {
        currentState = AVOIDING_BORDER;
    }
    

}

void stateMachine(){
  switch (currentState) {

    case MOVING_AROUND:
        // Move forward
        moveAround();
        break;

    case AVOIDING_BORDER:
        // Stop and turn away from the border
        stopMotors();
        moveBackward();
        delay(300);

        if (borderTriggered == 1) {
            // Left border detected, turn right
            turnRight();
        } else if (borderTriggered == 2) {
            // Right border detected, turn left
            turnLeft();
        } else if (borderTriggered == 3) {
            // Center border detected, turn randomly
            if (random(0, 2) == 0) {
                turnLeft();
            } else {
                turnRight();
            }
        }
        delay(1000); // adjust the delay
        
        stopMotors();
        delay(50);
        borderTriggered = 0; // Reset border trigger
        currentState = MOVING_AROUND; // Resume moving around
        break;

    case PLOWING_SNOW:
        // Logic for plowing snow cubes
        break;

    case IDLE:
        // Do nothing
        stopMotors();
        break;

  }
}


void obstacleDetection(){
    // Use ultrasonic and infrared sensors to detect obstacles
    // The ultrasonic at top to detect large obstacles
    // The infrared at low position to detect small obstacles(snow cubes)
    // At a specific range if both sensors or only low sensor detect the obstacle, there is a snow cube.
    // If only the top ultrasonic sensor detect the obstacle, there is a large obstacle.
}



void setup() {
  
  Serial.begin(9600); 
  Serial.println("Setup starting...");
  randomSeed(millis());

  // Initialize IMU
  Wire.begin();
  if (!imu.init()) {
      Serial.println("Failed to detect and initialize IMU!");
      while (1); 
  }
  imu.enableDefault();
  Serial.println("IMU initialized.");
  calibrateIMU();
 
  // Set motor control pins as outputs
  pinMode(L_F_DIR, OUTPUT);
  pinMode(L_F_PWM, OUTPUT);
  pinMode(L_R_DIR, OUTPUT);
  pinMode(L_R_PWM, OUTPUT);
  pinMode(R_F_DIR, OUTPUT);
  pinMode(R_F_PWM, OUTPUT);
  pinMode(R_R_DIR, OUTPUT);
  pinMode(R_R_PWM, OUTPUT);


  // watchdog timer setup
  WDT_Enable(WDT, WDT_MR_WDV(4000) | WDT_MR_WDFIEN | WDT_MR_WDRSTEN | WDT_MR_WDDBGHLT);
  
  currentState = MOVING_AROUND;
  
}

void loop() {
  // Reset Watchdog Timer
  WDT_Restart(WDT);
  
  readIMU();
  readLineSensors();
  
  checkBorders();
  stateMachine();
  
}
