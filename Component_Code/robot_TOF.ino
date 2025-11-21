#include <Wire.h>
#include <VL53L1X.h>

VL53L1X sensor;

void setup()
{
  Serial.begin(115200);
  
  
  delay(1500);

  Wire.begin();
  Wire.setClock(400000); 

  int retryCount = 0;
  while (!sensor.init())
  {
    Serial.print("Failed to detect sensor! Retrying (");
    Serial.print(retryCount++);
    Serial.println(")...");
    
    Wire.end();
    delay(100);
    Wire.begin();
    delay(100);
  }
  
  Serial.println("Sensor initialized successfully!");

 
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
    Serial.println("STOP");
    Serial.print(millis());
    Serial.println("ms");
  } else {

  }
}
