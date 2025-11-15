// -------------------------------------------------------------
// IR Obstacle Avoidance Sensor - Unit Test
// Detects obstacle presence and prints PASS/FAIL
// -------------------------------------------------------------

const int IR_PIN = 35;   // Digital output of IR sensor

void setup() {
  Serial.begin(9600);
  pinMode(IR_PIN, INPUT);

  Serial.println("=== IR Obstacle Sensor Unit Test ===");
  Serial.println("Place an obstacle in front of the sensor.");
  Serial.println("Sensor reading LOW = Obstacle detected.");
  Serial.println("-------------------------------------");
}

void loop() {
  int sensorValue = digitalRead(IR_PIN);

  if (sensorValue == LOW) {
    Serial.println("PASS: Obstacle detected ✔");
  } else {
    Serial.println("FAIL: No obstacle ✘");
  }

  delay(500); // slow down output
}