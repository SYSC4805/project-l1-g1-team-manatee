#include <Wire.h>
#include <VL53L1X.h>

VL53L1X sensor;

// Test parameters
const int MIN_VALID_MM = 50;     
const int MAX_VALID_MM = 1000;   
const int NOISE_THRESHOLD_MM = 30; 
int lastDistance = -1;

void printTestResult(const char* label, bool pass) {
  Serial.print(label);
  Serial.print(": ");
  Serial.println(pass ? "PASS" : "FAIL");
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  delay(500);

  bool init_ok = sensor.init();
  printTestResult("Sensor Initialization", init_ok);

  if (!init_ok) {
    Serial.println("Test aborted.");
    while (1);
  }

  sensor.setDistanceMode(VL53L1X::Long);
  sensor.setMeasurementTimingBudget(50000); // 50ms
  sensor.startContinuous(50);

  Serial.println("Reading distance");
  Serial.println(" ");
}

void loop() {
  int d = sensor.read();

  bool valid_read = d > 0;
  printTestResult("Read Distance", valid_read);

  if (!valid_read) return;

  Serial.print("Distance (mm): ");
  Serial.println(d);

  bool in_range = (d >= MIN_VALID_MM && d <= MAX_VALID_MM);
  printTestResult("Range Validity", in_range);

  bool stable = true;
  if (lastDistance != -1) {
    int diff = abs(d - lastDistance);
    if (diff > NOISE_THRESHOLD_MM) stable = false;
  }
  printTestResult("Stability Test", stable);

  lastDistance = d;

  delay(500);
}