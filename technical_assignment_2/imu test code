#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  Serial.println("Initializing MPU6050...");
  mpu.initialize();

  if (mpu.testConnection()) {
    Serial.println("MPU6050 connection successful.");
  } else {
    Serial.println("MPU6050 connection failed. Check wiring and ensure AD0 is connected to ground.");
    while (1); // Halt execution if sensor is not found.
  }
}

void loop() {
  int16_t ax, ay, az, gx, gy, gz;
  
  // Retrieve raw accelerometer and gyroscope measurements.
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Print raw sensor values to Serial Monitor.
  Serial.print("aX = "); Serial.print(ax);
  Serial.print(" | aY = "); Serial.print(ay);
  Serial.print(" | aZ = "); Serial.print(az);
  Serial.print(" || gX = "); Serial.print(gx);
  Serial.print(" | gY = "); Serial.print(gy);
  Serial.print(" | gZ = "); Serial.println(gz);

  delay(500); // Update every 500 ms.
}
