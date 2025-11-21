#include <Wire.h>
#include <VL53L1X.h>

VL53L1X sensor;

void setup()
{
  while (!Serial) {}
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000); // 400 kHz I2C

  sensor.setTimeout(500);
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1);
  }


  sensor.setDistanceMode(VL53L1X::Short);
  sensor.setMeasurementTimingBudget(20000); // minimum for short mode = 20 ms

  sensor.startContinuous(20); // measure every 20 ms
}

void loop()
{
  int distance = sensor.read(); // mm

  if (sensor.timeoutOccurred()) { 
    Serial.println("TIMEOUT"); 
    return;
  }

  if (distance > 0 && distance < 50) {
    stopMotors();
  } else {

  }
}
