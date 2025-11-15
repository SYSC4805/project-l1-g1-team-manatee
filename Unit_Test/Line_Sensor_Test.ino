// -------------------------------------------------------------
// 3-Sensor Line Follower - Unit Test
// Detects black tape over a white surface
// -------------------------------------------------------------

const int LEFT_SENSOR_PIN   = 11;
const int CENTER_SENSOR_PIN = 12;
const int RIGHT_SENSOR_PIN  = 13;

void setup() {
  Serial.begin(9600);

  pinMode(LEFT_SENSOR_PIN, INPUT);
  pinMode(CENTER_SENSOR_PIN, INPUT);
  pinMode(RIGHT_SENSOR_PIN, INPUT);

  Serial.println("=== 3-Sensor Line Follower Unit Test ===");
  Serial.println("LOW = Black line detected");
  Serial.println("HIGH = White surface detected");
  Serial.println("-------------------------------------");
}

void loop() {
  int left   = digitalRead(LEFT_SENSOR_PIN);
  int center = digitalRead(CENTER_SENSOR_PIN);
  int right  = digitalRead(RIGHT_SENSOR_PIN);

  Serial.print("Left: ");
  Serial.print(left == LOW ? "BLACK O" : "WHITE X");
  Serial.print(" | Center: ");
  Serial.print(center == LOW ? "BLACK O" : "WHITE X");
  Serial.print(" | Right: ");
  Serial.println(right == LOW ? "BLACK O" : "WHITE X");

  delay(500); // update every 0.5s
}