volatile unsigned long t = 0;
volatile unsigned long t_old = 0;
volatile boolean CaptureFlag = 0;

unsigned long lastPrint = 0;
unsigned long pulsePeriod = 0;

float rpm = 0;

void setup() {
  Serial.begin(115200);
  pinMode(2, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(2), time_ISR, FALLING);
}

void loop() {

  if (CaptureFlag) {
    CaptureFlag = 0;

    pulsePeriod = t - t_old;
    t_old = t;

    if (pulsePeriod >= 25) {
      rpm = (1000.0 / pulsePeriod) * 60.0;
    } else {
      rpm = 0;  // noise pulses ignored
    }

    Serial.print("Pulse Period (ms): ");
    Serial.print(pulsePeriod);

    bool pass = (pulsePeriod >= 25 && pulsePeriod <= 200);
    Serial.print(" | Test: ");
    Serial.println(pass ? "PASS" : "FAIL");

    Serial.print("RPM: ");
    Serial.println(rpm, 1);

    Serial.println("---------------------");
  }
}

void time_ISR() {
  unsigned long now = millis();

  if (now - t_old > 25) {
    t = now;
    CaptureFlag = 1;
  }
}