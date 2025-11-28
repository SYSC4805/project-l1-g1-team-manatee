#include <Arduino.h>

// --- Pin Definitions for Line Follower Sensors ---
const int LINE_SENSOR_LEFT = A2; 
const int LINE_SENSOR_CENTER = A1; 
const int LINE_SENSOR_RIGHT = A0; 

const int LINE_SENSOR_LEFT2 = A5; 
const int LINE_SENSOR_CENTER2 = A4; 
const int LINE_SENSOR_RIGHT2 = A3; 

// --- Global Variables for Sensor Values ---
int leftValue = 0;
int centerValue = 0;
int rightValue = 0;
int leftValue2 = 0;
int centerValue2 = 0;
int rightValue2 = 0;

/**
 * @brief Reads the analog values from the three line follower sensors 
 *        and stores them in global variables.
 */
void readLineSensors(){
    leftValue = analogRead(LINE_SENSOR_LEFT);
    centerValue = analogRead(LINE_SENSOR_CENTER);
    rightValue = analogRead(LINE_SENSOR_RIGHT);
    leftValue2 = analogRead(LINE_SENSOR_LEFT2);
    centerValue2 = analogRead(LINE_SENSOR_CENTER2);
    rightValue2 = analogRead(LINE_SENSOR_RIGHT2);
    
    // Debugging: 
  
    Serial.print("L:"); Serial.print(leftValue2);
    Serial.print(" C:"); Serial.print(centerValue2);
    Serial.print(" R:"); Serial.println(rightValue2);
}


// --- Setup Function ---
void setup() {
  // Start serial communication for debugging
  Serial.begin(9600);
  Serial.println("Line Sensor Raw Value Test");
  Serial.println("Move sensors over white/black surfaces to see readings.");
  Serial.println("---------------------------------");

  // Set sensor pins to INPUT mode
  pinMode(LINE_SENSOR_LEFT, INPUT);
  pinMode(LINE_SENSOR_CENTER, INPUT);
  pinMode(LINE_SENSOR_RIGHT, INPUT);
}

// --- Main Loop for Testing ---
void loop() {
  // Update the sensor values by calling the function
  readLineSensors();

  // Print the values in a single, clean line
  Serial.print("L: ");
  Serial.print(leftValue);
  Serial.print("\t C: "); // \t is a tab character for spacing
  Serial.print(centerValue);
  Serial.print("\t R: ");
  Serial.println(rightValue);

  // Wait for a short period before the next reading
  delay(300);
}
