#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>
#include "robot_pins.h"

// --- Sensors ---
VL53L1X tof;

// --- Thresholds ---
const int LINE_THRESHOLD = 850; // Calibrate based on environment to detect line

// --- Ultrasonic helper ---
float readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000);
  if (duration == 0) return -1;
  return (duration * 0.0343) / 2.0; // cm
}

void setup() {
  Serial.begin(9600);

  // Line sensors
  pinMode(LINE_SENSOR_LEFT, INPUT);
  pinMode(LINE_SENSOR_CENTER, INPUT);
  pinMode(LINE_SENSOR_RIGHT, INPUT);

  // Ultrasonics
  pinMode(ULTRA1_TRIG_PIN, OUTPUT);
  pinMode(ULTRA1_ECHO_PIN, INPUT);
  pinMode(ULTRA2_TRIG_PIN, OUTPUT);
  pinMode(ULTRA2_ECHO_PIN, INPUT);

  // ToF init
  pinMode(TOF_XSHUT, OUTPUT);
  digitalWrite(TOF_XSHUT, LOW); delay(10);
  digitalWrite(TOF_XSHUT, HIGH); delay(10);

  Wire.begin();
  if (!tof.init()) {
    Serial.println("Failed to detect VL53L1X!");
    while (1);
  }
  tof.setTimeout(500);
  tof.setDistanceMode(VL53L1X::Short);
  tof.setMeasurementTimingBudget(20000);
  tof.startContinuous(50);

  Serial.println("Multi-sensor integration test starting...");
}

void loop() {
  // --- Line sensors ---
  int leftVal   = analogRead(LINE_SENSOR_LEFT);
  int centerVal = analogRead(LINE_SENSOR_CENTER);
  int rightVal  = analogRead(LINE_SENSOR_RIGHT);

  // --- Ultrasonics ---
  float ultraLeft  = readUltrasonic(ULTRA1_TRIG_PIN, ULTRA1_ECHO_PIN);
  delay(50); // stagger ultrasonic reads
  float ultraRight = readUltrasonic(ULTRA2_TRIG_PIN, ULTRA2_ECHO_PIN);

  // --- ToF ---
  int rawToF = tof.read();
  float tofDist = (tof.timeoutOccurred()) ? -1 : rawToF / 10.0; // mm → cm

  // --- Print raw sensor values ---
  Serial.print("Line L:"); Serial.print(leftVal);
  Serial.print(" C:"); Serial.print(centerVal);
  Serial.print(" R:"); Serial.print(rightVal);

  Serial.print(" | UltraL:"); Serial.print(ultraLeft);
  Serial.print(" cm UltraR:"); Serial.print(ultraRight);
  Serial.print(" cm");

  Serial.print(" | ToF:"); Serial.print(tofDist);
  Serial.println(" cm");

  if (leftVal <= LINE_THRESHOLD){
    Serial.println("LINE LEFT");
  }
  if (centerVal <= LINE_THRESHOLD){
    Serial.println("LINE CENTER");
  }
    if (rightVal <= LINE_THRESHOLD){
    Serial.println("LINE RIGHT");
  }
  
  delay(2000); // Delay before next reading
}