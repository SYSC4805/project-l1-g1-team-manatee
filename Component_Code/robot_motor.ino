#include <Arduino.h>

// --- Pin Definitions for Motors ---
const int L_F_DIR = 1; // Left Front Motor Direction Pin
const int L_F_PWM = 2; // Left Front Motor PWM Pin

const int L_R_DIR = 3; // Left Rear Motor Direction Pin
const int L_R_PWM = 4; // Left Rear Motor PWM Pin

const int R_F_DIR = 5; // Right Front Motor Direction Pin
const int R_F_PWM = 7; // Right Front Motor PWM Pin

const int R_R_DIR = 6; // Right Rear Motor Direction Pin
const int R_R_PWM = 8; // Right Rear Motor PWM Pin

// --- Constants for Motor Control ---
const int MAX_SPEED = 255;      // The max PWM value for analogWrite
const int FORWARD_SPEED = 200; // A PWM value for forward movement (0-255)
const int TURN_SPEED = 150;    // A PWM value for turning (0-255)

// --- Core Motor Control Functions ---


void setMotorSpeed(int dirPin, int pwmPin, int speed_pwm) {
  // Set the direction based on the sign of the speed
  if (speed_pwm >= 0) {
    digitalWrite(dirPin, HIGH); // Set one direction
  } else {
    digitalWrite(dirPin, LOW);  // Set the opposite direction
  }
  
  // Write the absolute PWM value to the motor
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

// --- Setup Function ---
void setup() {
  // Start serial communication for debugging
  Serial.begin(9600);
  Serial.println("Motor Test Program Starting...");

  // Set all motor control pins to OUTPUT mode
  pinMode(L_F_DIR, OUTPUT);
  pinMode(L_F_PWM, OUTPUT);
  pinMode(L_R_DIR, OUTPUT);
  pinMode(L_R_PWM, OUTPUT);
  pinMode(R_F_DIR, OUTPUT);
  pinMode(R_F_PWM, OUTPUT);
  pinMode(R_R_DIR, OUTPUT);
  pinMode(R_R_PWM, OUTPUT);

  Serial.println("Setup complete. Starting movement sequence.");
}

// --- Main Loop for Testing ---
void loop() {
  // This loop will execute a sequence of movements to test all motor functions.
  
  Serial.println("Moving forward for 2 seconds...");
  moveForward();
  delay(2000);

  Serial.println("Stopping for 1 second...");
  stopMotors();
  delay(1000);

  Serial.println("Moving backward for 2 seconds...");
  moveBackward();
  delay(2000);

  Serial.println("Stopping for 1 second...");
  stopMotors();
  delay(1000);

  Serial.println("Turning left for 1 second...");
  turnLeft();
  delay(1000);

  Serial.println("Stopping for 1 second...");
  stopMotors();
  delay(1000);

  Serial.println("Turning right for 1 second...");
  turnRight();
  delay(1000);

  Serial.println("Stopping for 3 seconds... Sequence will repeat.");
  stopMotors();
  delay(3000);
}