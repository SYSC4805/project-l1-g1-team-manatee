#ifndef ROBOT_PINS_H
#define ROBOT_PINS_H

#include <Arduino.h>

// --- Motor Pins ---
const int L_F_DIR = 51; // Left Front Motor Direction
const int L_F_PWM = 47; // Left Front Motor PWM
const int L_R_DIR = 46; // Left Rear Motor Direction
const int L_R_PWM = 50; // Left Rear Motor PWM
const int R_F_DIR = 53; // Right Front Motor Direction
const int R_F_PWM = 49; // Right Front Motor PWM
const int R_R_DIR = 48; // Right Rear Motor Direction
const int R_R_PWM = 52; // Right Rear Motor PWM

// --- Line Sensor Pins ---
const int LINE_SENSOR_LEFT   = A2;
const int LINE_SENSOR_CENTER = A1;
const int LINE_SENSOR_RIGHT  = A0;

// --- Time-of-Flight (ToF) Sensor Pins ---
const int TOF_XSHUT = 25; // XSHUT pin for ToF sensor
const int TOF_SCL = 21;
const int TOF_SDA = 20;

// --- Infrared Sensor Pin ---
const int IR_FRONT_PIN = 35; // Front IR sensor

// --- Ultrasonic Sensor 1 ---
const int ULTRA1_TRIG_PIN = 5;
const int ULTRA1_ECHO_PIN = 4;


// --- Ultrasonic Sensor 2 ---
const int ULTRA2_TRIG_PIN = 8;
const int ULTRA2_ECHO_PIN = 9;

#endif // ROBOT_PINS_H