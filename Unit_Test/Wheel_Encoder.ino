#define ENCODER_PIN 2

volatile unsigned long t = 0;
volatile unsigned long t_old = 0;
volatile boolean captureFlag = 0;

const int MIN_VALID_MS = 25;     // Debounce threshold (ignore noise)
const int MAX_VALID_MS = 2000;   // Wheel cannot reasonably be slower
const int RPM_TOOTH_COUNT = 1;   // Change if more teeth per revolution

unsigned long pulsePeriod = 0;
float rpm = 0;

void printResult(const char* label, bool pass) {
  Serial.print(label);
  Serial.print(": ");
  Serial.println(pass ? "PASS" : "FAIL");
}

void setup() {
  Serial.begin(115200);

  pinMode(ENCODER_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), encoderISR, FALLING);

}

void loop() {

  if (captureFlag) {
    captureFlag = 0;

    pulsePeriod = t - t_old;
    t_old = t;

    Serial.print("Pulse Period (ms): ");
    Serial.println(pulsePeriod);

    bool pulseDetected = (pulsePeriod > 0);
    printResult("Pulse Detected", pulseDetected);

    bool validTiming = (pulsePeriod >= MIN_VALID_MS && pulsePeriod <= MAX_VALID_MS);
    printResult("Timing Validity", validTiming);

    if (validTiming) {
      rpm = (1000.0 / pulsePeriod) * 60.0 / RPM_TOOTH_COUNT;
    } else {
      rpm = 0;
    }

    Serial.print("RPM: ");
    Serial.println(rpm, 2);

  }
}

void encoderISR() {
  unsigned long now = millis();

  if (now - t_old > MIN_VALID_MS) {
    t = now;
    captureFlag = 1;
  }
}