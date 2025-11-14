#include <Arduino.h>

// --- Pin Definitions for Sensors ---

// Ultrasonic Sensor Pins
const int ULTRASONIC_TRIG_PIN = 12; // Trigger pin for the ultrasonic sensor
const int ULTRASONIC_ECHO_PIN = 11; // Echo pin for the ultrasonic sensor

// Infrared (IR) Obstacle Sensor Pin
const int IR_SENSOR_PIN = 10;       // Digital OUT pin of the IR sensor

// --- Global Variables ---
long duration; // Stores the echo pulse duration
float distance; // Stores the calculated distance in cm

void setup() {
  // Start serial communication for debugging
  Serial.begin(9600);
  Serial.println("Ultrasonic and IR Sensor Test Program");
  Serial.println("-------------------------------------");

  // --- IMPORTANT VOLTAGE WARNING ---
  Serial.println("WARNING: Arduino Due operates at 3.3V.");
  Serial.println("Ensure your sensors are 3.3V compatible or use a level shifter!");
  Serial.println("-------------------------------------");

  // Configure sensor pins
  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);
  pinMode(IR_SENSOR_PIN, INPUT);
}

void loop() {
  // --- Test Ultrasonic Sensor ---

  // 1. Clear the trigPin by setting it LOW
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  delayMicroseconds(2);

  // 2. Generate a 10 microsecond pulse on the trigPin to trigger a measurement
  digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

  // 3. Read the echoPin. pulseIn() returns the pulse duration in microseconds.
  duration = pulseIn(ULTRASONIC_ECHO_PIN, HIGH);

  // 4. Calculate the distance in centimeters.
  // Speed of sound is ~343 m/s or 0.0343 cm/µs.
  // The pulse travels to the object and back, so we divide the total time by 2.
  distance = (duration * 0.0343) / 2;

  // Print the ultrasonic sensor reading
  Serial.print("Ultrasonic Distance: ");
  if (distance > 400 || distance <= 0) {
    Serial.println("Out of range");
  } else {
    Serial.print(distance);
    Serial.println(" cm");
  }


  // --- Test IR Sensor ---

  // Read the digital value from the IR sensor.
  // Most modules output LOW when an obstacle is detected, and HIGH when clear.
  int ir_status = digitalRead(IR_SENSOR_PIN);

  // Print the IR sensor status
  Serial.print("IR Sensor Status:    ");
  if (ir_status == LOW) {
    Serial.println("Obstacle DETECTED");
  } else {
    Serial.println("Clear");
  }

  Serial.println("-------------------------------------");