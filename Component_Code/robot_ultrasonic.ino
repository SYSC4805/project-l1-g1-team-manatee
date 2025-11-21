// Ultrasonic Sensor (HC-SR04) Example
// Prints "Detected" if obstacle < 5 cm

const int trigPin = 26;
const int echoPin = 27;

const int trigPin2 = 28;
const int echoPin2 = 29;

void setup() {
  Serial.begin(9600);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
}

void loop() {
  // Send ultrasonic pulse
  digitalWrite(trigPin, LOW);
  digitalWrite(trigPin2, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  digitalWrite(trigPin2, LOW);

  // Read echo time
  long duration = pulseIn(echoPin, HIGH);
  long duration2 = pulseIn(echoPin2, HIGH);

  // Convert to distance in cm (speed of sound formula)
  float distance = duration * 0.0343 / 2;
  float distance2 = duration2 * 0.0343 / 2;


  // Check for obstacle within 5 cm
  if ((distance > 0 && distance < 5) || (distance2 > 0 && distance2 < 5)) {
    stopMotors();
  } else {
  }

  delay(200);
}