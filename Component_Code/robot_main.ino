#include <Arduino.h>
#include <VL53L1X.h>
#include <Wire.h>
VL53L1X tof;

// --- 1. Pin Definitions ---

// Motor Pins
const int L_F_DIR = 51; // Left Front Motor Direction
const int L_F_PWM = 47; // Left Front Motor PWM
const int L_R_DIR = 46; // Left Rear Motor Direction
const int L_R_PWM = 50; // Left Rear Motor PWM
const int R_F_DIR = 53; // Right Front Motor Direction
const int R_F_PWM = 49; // Right Front Motor PWM
const int R_R_DIR = 48; // Right Rear Motor Direction
const int R_R_PWM = 52; // Right Rear Motor PWM

// Line Sensor Pins
const int LINE_SENSOR_LEFT = A2; 
const int LINE_SENSOR_CENTER = A1; 
const int LINE_SENSOR_RIGHT = A0; 

const int LINE_SENSOR_LEFT2 = A3; 
const int LINE_SENSOR_CENTER2 = A4; 
const int LINE_SENSOR_RIGHT2 = A5; 


// Ultrasonic Sensor Pins
const int trigPin = 5;
const int echoPin = 4;

// IR Sensor Pin
const int sensorPin_IR = 35; 
const int sensorPin_IR2 = 13; 


// --- 2. Constants & Variables ---

// Motor Speeds
const int MAX_SPEED = 255;      // Max PWM
const int FORWARD_SPEED = 128; // Default forward speed
const int TURN_SPEED = 170;     // Turning speed (increased slightly for better torque)

// Line Sensor Thresholds
const int BORDER_THRESHOLD = 850; 
int leftValue = 0;
int centerValue = 0;
int rightValue = 0;

int leftValue2 = 0;
int centerValue2 = 0;
int rightValue2 = 0;

int borderTriggered = 0; // 1-left, 2-right, 3-center

const int tofThreshold = 20; // cm



// Robot States
enum RobotState {
    IDLE,
    MOVING_AROUND,   // Default moving state
    AVOIDING_BORDER, // Triggered by line sensor
    AVOIDING_OBSTACLE // Triggered by obstacle sensor
};
RobotState currentState = IDLE; // Start in IDLE or MOVING_AROUND based on preference

// --- 3. Motor Functions ---

void setMotorSpeed(int dirPin, int pwmPin, int speed_pwm) {
  // Set direction
  if (speed_pwm >= 0) {
    digitalWrite(dirPin, LOW); 
  } else {
    digitalWrite(dirPin, HIGH);  
  }
  // Set PWM
  analogWrite(pwmPin, abs(speed_pwm));
}

void setLeftMotor(int speed_pwm) {
    setMotorSpeed(L_F_DIR, L_F_PWM, speed_pwm);
    setMotorSpeed(L_R_DIR, L_R_PWM, speed_pwm);
}

void setRightMotor(int speed_pwm) {
    setMotorSpeed(R_F_DIR, R_F_PWM, speed_pwm);
    setMotorSpeed(R_R_DIR, R_R_PWM, speed_pwm);
}

void moveForward() {
    setLeftMotor(FORWARD_SPEED);
    setRightMotor(FORWARD_SPEED);
}

void moveBackward() {
    setLeftMotor(-FORWARD_SPEED);
    setRightMotor(-FORWARD_SPEED);
}

void turnLeft() {
    setLeftMotor(-TURN_SPEED);
    setRightMotor(TURN_SPEED);
}

void turnRight() {
    setLeftMotor(TURN_SPEED);  
    setRightMotor(-TURN_SPEED);
}

void stopMotors() {
    setLeftMotor(0);
    setRightMotor(0);
} 

void moveAround(){
    moveForward();
}

// --- 4. Sensor Functions ---

void readLineSensors(){
    leftValue = analogRead(LINE_SENSOR_LEFT);
    centerValue = analogRead(LINE_SENSOR_CENTER);
    rightValue = analogRead(LINE_SENSOR_RIGHT);
    leftValue2 = analogRead(LINE_SENSOR_LEFT2);
    centerValue2 = analogRead(LINE_SENSOR_CENTER2);
    rightValue2 = analogRead(LINE_SENSOR_RIGHT2);
    
    // Debugging: 
    Serial.print("L:"); Serial.print(leftValue);
    Serial.print(" C:"); Serial.print(centerValue);
    Serial.print(" R:"); Serial.println(rightValue);
    Serial.print("L:"); Serial.print(leftValue2);
    Serial.print(" C:"); Serial.print(centerValue2);
    Serial.print(" R:"); Serial.println(rightValue2);
}

void checkBorders(){
    // Only check borders if we are currently moving normally
    if (currentState != MOVING_AROUND) {
        return; 
    }
    
    borderTriggered = 0; 

    // Logic: Assuming HIGHER value means LINE DETECTED (Black line usually gives high analog value on some sensors)
    // If your sensor gives LOW value on black line, change '>' to '<'
    if ((leftValue > BORDER_THRESHOLD) || (leftValue2 > BORDER_THRESHOLD)) {
        borderTriggered = 1; // Left
    } else if ((rightValue > BORDER_THRESHOLD)|| (rightValue2 > BORDER_THRESHOLD)){
        borderTriggered = 2; // Right
    } else if ((centerValue > BORDER_THRESHOLD) || (centerValue2 > BORDER_THRESHOLD)) {
        borderTriggered = 3; // Center
    }
    
    if (borderTriggered != 0) {
        currentState = AVOIDING_BORDER;
    }
}
bool checkObstacles(){
    // 1. Ultrasonic Sensor
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    long duration = pulseIn(echoPin, HIGH, 18000); // 18ms timeout
    float ultrasonicDistance = duration * 0.034 / 2;

    if (duration > 0 && ultrasonicDistance < 30) { // 30cm threshold
         return true;
    }

    // 2. IR Sensor
    if ((digitalRead(sensorPin_IR) == LOW) || (digitalRead(sensorPin_IR2) == LOW)) { // 假设 LOW 意味着检测到障碍物
        return true;
    }

    // 3. ToF Sensor
     int rawTof = tof.read();
    float tofDistance = (tof.timeoutOccurred()) ? -1.0 : rawTof / 10.0; // Convert mm to cm

    if (tofDistance > 0 && tofDistance < tofThreshold) {
        return true;
    }
    return false;
}

// --- 5. State Machine ---

void stateMachine(){
  switch (currentState) {

    case MOVING_AROUND:
        checkBorders();
        moveAround();
        checkBorders();
        delay(280);
        checkBorders();

        stopMotors();
        checkBorders();
        delay(30);
        
        // Priority: Border > Obstacle
        // Only check obstacles if border hasn't triggered a state change
        if (currentState != AVOIDING_BORDER) {
            if (checkObstacles()) {
                currentState = AVOIDING_OBSTACLE;
            }
        }
        break;

    case AVOIDING_BORDER:
        Serial.println("Border Detected! Avoiding...");
        stopMotors();
        delay(600); // Short pause
        
        // 1. Back up
        moveBackward();
        delay(1000); 

        // 2. Turn away
        if (borderTriggered == 1) {
            // Left sensor hit -> Turn Right
            turnRight();
        } else if (borderTriggered == 2) {
            // Right sensor hit -> Turn Left
            turnLeft();
        } else {
            // Center hit -> Random Turn
            //if (random(0, 2) == 0) turnLeft(); else turnRight();
            turnRight();
        }
        delay(1500); // Turn duration
        
        stopMotors();
        delay(100);
        
        // 3. Reset
        borderTriggered = 0; 
        currentState = MOVING_AROUND; 
        break;
    case AVOIDING_OBSTACLE:
        Serial.println("Obstacle Detected! Avoiding...");
        

        stopMotors();
        delay(100);
        
        moveBackward();
        delay(100); 

        if (random(0, 2) == 0) turnLeft(); else turnRight();
        delay(500); 
        
        stopMotors();
        delay(100);
        
        currentState = MOVING_AROUND; 
        break;

    case IDLE:
        stopMotors();
        break;
  }
}

// --- 6. Setup & Loop ---

void setup() {
  Serial.begin(9600); 
  Serial.println("Motor & Line Sensor Test Starting...");
  
  randomSeed(analogRead(A5)); // Use an unconnected pin for random seed

  // Motor Pins Setup
  pinMode(L_F_DIR, OUTPUT); pinMode(L_F_PWM, OUTPUT);
  pinMode(L_R_DIR, OUTPUT); pinMode(L_R_PWM, OUTPUT);
  pinMode(R_F_DIR, OUTPUT); pinMode(R_F_PWM, OUTPUT);
  pinMode(R_R_DIR, OUTPUT); pinMode(R_R_PWM, OUTPUT);

  // Ultrasonic Sensor Pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // IR Sensor Pin
  pinMode(sensorPin_IR, INPUT);

  pinMode(25,OUTPUT);
  digitalWrite(25,LOW);
  delay(10);
  digitalWrite(25,HIGH);
  delay(10);
  Wire.begin();
  tof.setTimeout(500);
  tof.setDistanceMode(VL53L1X::Short);
  tof.setMeasurementTimingBudget(20000);
  tof.startContinuous(50);

  pinMode(sensorPin_IR2, INPUT);

  pinMode(13,OUTPUT);
  digitalWrite(13,LOW);
  delay(10);
  digitalWrite(13,HIGH);
  delay(10);
  Wire.begin();
  tof.setTimeout(500);
  tof.setDistanceMode(VL53L1X::Short);
  tof.setMeasurementTimingBudget(20000);
  tof.startContinuous(50);



  // Start moving immediately for test
  currentState = MOVING_AROUND; 
}

void loop() {
    
    
  readLineSensors();

  
  stateMachine();
  
  // Small delay to stabilize loop
  delay(10);
}
