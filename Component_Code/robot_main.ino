#include <Arduino.h>

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

// --- 2. Constants & Variables ---

// Motor Speeds
const int MAX_SPEED = 255;      // Max PWM
const int FORWARD_SPEED = 128;  // Default forward speed
const int TURN_SPEED = 170;     // Turning speed (increased slightly for better torque)

// Line Sensor Thresholds
const int BORDER_THRESHOLD = 875; 
int leftValue = 0;
int centerValue = 0;
int rightValue = 0;
int borderTriggered = 0; // 1-left, 2-right, 3-center

// Robot States
enum RobotState {
    IDLE,
    MOVING_AROUND,   // Default moving state
    AVOIDING_BORDER  // Triggered by line sensor
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
    
    // Debugging: 
    Serial.print("L:"); Serial.print(leftValue);
    Serial.print(" C:"); Serial.print(centerValue);
    Serial.print(" R:"); Serial.println(rightValue);
}

void checkBorders(){
    // Only check borders if we are currently moving normally
    if (currentState != MOVING_AROUND) {
        return; 
    }
    
    borderTriggered = 0; 

    // Logic: Assuming HIGHER value means LINE DETECTED (Black line usually gives high analog value on some sensors)
    // If your sensor gives LOW value on black line, change '>' to '<'
    if (leftValue > BORDER_THRESHOLD) {
        borderTriggered = 1; // Left
    } else if (rightValue > BORDER_THRESHOLD) {
        borderTriggered = 2; // Right
    } else if (centerValue > BORDER_THRESHOLD) {
        borderTriggered = 3; // Center
    }
    
    if (borderTriggered != 0) {
        currentState = AVOIDING_BORDER;
    }
}

// --- 5. State Machine ---

void stateMachine(){
  switch (currentState) {

    case MOVING_AROUND:
        moveAround();
        checkBorders();
        break;

    case AVOIDING_BORDER:
        Serial.println("Border Detected! Avoiding...");
        stopMotors();
        delay(100); // Short pause
        
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
        delay(2000); // Turn duration
        
        stopMotors();
        delay(100);
        
        // 3. Reset
        borderTriggered = 0; 
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

  // Start moving immediately for test
  currentState = MOVING_AROUND; 
}

void loop() {
  readLineSensors();
  
  stateMachine();
  
  // Small delay to stabilize loop
  delay(10);
}
