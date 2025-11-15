const int IR_PIN = 35;   

void setup() {
  Serial.begin(9600);
  pinMode(IR_PIN, INPUT);
}

void loop() {
  int sensorValue = digitalRead(IR_PIN);

  if (sensorValue == LOW) {
    Serial.println("PASS: Obstacle detected");
  } else {
    Serial.println("FAIL: No obstacle");
  }

  delay(500); 
}