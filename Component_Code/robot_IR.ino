const int sensorPin_IR = 35; 

void setup() {
  Serial.begin(9600);
  pinMode(sensorPin_IR, INPUT);
}

void loop() {
  int sensorValue = digitalRead(sensorPin);

  if (sensorValue == LOW) {
    stopMotors();
  } else {

  }

  delay(200); 
}