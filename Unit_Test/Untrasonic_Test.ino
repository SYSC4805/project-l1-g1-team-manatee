#define TRIG_PIN 26
#define ECHO_PIN 27

const float SOUND_SPEED = 0.0343f; // cm/us
const int MIN_VALID_CM = 3;        // physical limit of HC-SR04
const int MAX_VALID_CM = 100;      // 4 meters
const int STABILITY_THRESHOLD = 10; // cm variation allowed

long lastDistance = -1;

void printResult(const char* testName, bool status) {
  Serial.print(testName);
  Serial.print(": ");
  Serial.println(status ? "PASS" : "FAIL");
}

void setup() {
  Serial.begin(115200);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

}

long readDistanceCM() {
  // Ensure trigger is LOW
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(5);

  // Trigger pulse: 10 microseconds HIGH
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Measure echo pulse width
  long duration = pulseIn(ECHO_PIN, HIGH, 30000); // 30ms timeout (~5m range)

  if (duration == 0) return -1; // No echo received
  return duration * SOUND_SPEED / 2.0;
}

void loop() {
  long d = readDistanceCM();

  Serial.print("Distance (cm): ");
  if (d == -1) {
    Serial.println("NO ECHO");
  } else {
    Serial.println(d);
  }

  bool echoOK = (d != -1);
  printResult("Echo Detection", echoOK);

  bool rangeOK = (d >= MIN_VALID_CM && d <= MAX_VALID_CM);
  printResult("Range Validity", rangeOK);

  bool stable = true;
  if (lastDistance != -1 && d != -1) {
    if (abs(d - lastDistance) > STABILITY_THRESHOLD) stable = false;
  }
  printResult("Reading Stability", stable);

  lastDistance = d;

  delay(500);
}