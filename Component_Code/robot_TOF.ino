#include <Wire.h>
#include <VL53L1X.h>

VL53L1X sensor;
const int XSHUT_PIN = 25; // Connect sensor XSHUT

void setup()
{
  Serial.begin(9600);

  // --- HARDWARE RESET SEQUENCE ---
  // This forces the sensor to shut down and restart
  pinMode(XSHUT_PIN, OUTPUT);
  digitalWrite(XSHUT_PIN, LOW); // Turn sensor OFF
  delay(10);                    // Wait for it to fully discharge
  digitalWrite(XSHUT_PIN, HIGH); // Turn sensor ON
  delay(10);                    // Wait for it to boot up
  
  Wire.begin();
  Wire.setClock(400000); 
  sensor.setTimeout(500);

  while (!sensor.init())
  {
    Serial.println("Failed to detect sensor! Retrying...");
    // If it fails, try hard resetting again
    digitalWrite(XSHUT_PIN, LOW);
    delay(10);
    digitalWrite(XSHUT_PIN, HIGH);
    delay(10);
  }
  
  Serial.println("Sensor initialized successfully!");
  
  sensor.setDistanceMode(VL53L1X::Short);
  sensor.setMeasurementTimingBudget(20000); 
  sensor.startContinuous(20); 
}

void loop() {
    
    int distance = sensor.read(); // mm



  if (sensor.timeoutOccurred()) {

    Serial.println("TIMEOUT");

    return;

  }



  if (distance > 0 && distance < 100) {

    Serial.println("object");

    Serial.print(millis());

    Serial.println("ms");

  } else {
  }

}
