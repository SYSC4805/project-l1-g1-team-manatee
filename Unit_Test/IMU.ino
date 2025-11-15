#include <Wire.h>
#include <LSM6.h>        // Gyro + Accelerometer
#include <LIS3MDL.h>     // Magnetometer

LSM6 imu;
LIS3MDL mag;

bool testAccelGyro();
bool testMag();

void printResult(const char* testName, bool result) {
  Serial.print(testName);
  Serial.print(": ");
  Serial.println(result ? "PASS" : "FAIL");
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  delay(1000); 

  bool ag   = testAccelGyro();
  printResult("Accelerometer/Gyroscope", ag);

  bool mg   = testMag();
  printResult("Magnetometer", mg);

}

void loop() {
  
}


bool testAccelGyro() {
  imu.enableDefault();
  delay(50);

  imu.read();

  // Basic sanity checks: values must not be static zeros
  bool accelOK = !(imu.a.x == 0 && imu.a.y == 0 && imu.a.z == 0);
  bool gyroOK  = !(imu.g.x == 0 && imu.g.y == 0 && imu.g.z == 0);

  Serial.print("Accel Raw: ");
  Serial.print(imu.a.x); Serial.print(", ");
  Serial.print(imu.a.y); Serial.print(", ");
  Serial.println(imu.a.z);

  Serial.print("Gyro Raw: ");
  Serial.print(imu.g.x); Serial.print(", ");
  Serial.print(imu.g.y); Serial.print(", ");
  Serial.println(imu.g.z);

  return accelOK && gyroOK;
}

bool testMag() {
  mag.enableDefault();
  delay(50);

  mag.read();

  bool magOK = !(mag.m.x == 0 && mag.m.y == 0 && mag.m.z == 0);

  Serial.print("Mag Raw: ");
  Serial.print(mag.m.x); Serial.print(", ");
  Serial.print(mag.m.y); Serial.print(", ");
  Serial.println(mag.m.z);

  return magOK;
}