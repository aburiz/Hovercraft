#include <Wire.h>
#include <Servo.h>
#include "MPU6050.h"

// Pin definitions based on controller pinout 
// Servo motor control: Controller P9 -> Arduino digital pin 9 (PWM)
#define SERVO_PIN 9  
// LED "L" / D3 for brightness control: Controller D3 -> Arduino digital pin 3 (PWM)
#define LED_PIN 11    

// Constants for servo control and LED brightness 
#define YAW_MAX 81   // Maximum allowed yaw angle in degrees
#define YAW_MIN -81  // Minimum allowed yaw angle in degrees

#define ACC_THRESHOLD_LOW  0.12   // g: below this, LED off
#define ACC_THRESHOLD_HIGH 1.12    // g: above this, LED full on

// Conversion factors for MPU6050 (default settings)
// In ±2g mode, sensitivity = 16384 LSB/g
// In ±250°/sec mode, sensitivity = 131 LSB/(°/sec)
#define ACCEL_SENSITIVITY 16384.0  
#define GYRO_SENSITIVITY  131.0     

// Create MPU6050 and Servo objects
MPU6050 mpu;
Servo servo;

// Variables for integrating gyro Z (yaw)
float yaw = 0.0;
unsigned long lastGyroTime = 0;

// Variables for distance integration along X-axis
float velocityX = 0.0;
float distanceX = 0.0;
unsigned long lastDistanceTime = 0;

// Output interval in milliseconds (once per second)
const unsigned long OUTPUT_INTERVAL = 1000;
unsigned long lastOutputTime = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  
  // Initialize MPU6050
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1);
  }
  
  // Configure full-scale ranges and sample rates (adjust per experiment)
  // For example, to set gyro to ±250°/sec and accelerometer to ±2g:
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  // Set sample rate divider if needed: mpu.setRate(sample_rate_divider);

  // Attach servo to specified pin
  servo.attach(SERVO_PIN);
  
  // Initialize LED pin as output
  pinMode(LED_PIN, OUTPUT);
  
  // Initialize timing variables
  lastGyroTime = millis();
  lastDistanceTime = millis();
  lastOutputTime = millis();
}

void loop() {
  // Read raw data from MPU6050: acceleration and gyroscope
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  // Convert raw accelerometer readings to g's
  float accelX = ax / ACCEL_SENSITIVITY;
  float accelY = ay / ACCEL_SENSITIVITY;
  float accelZ = az / ACCEL_SENSITIVITY;
  
  // Convert raw gyroscope readings to degrees per second
  float gyroX = gx / GYRO_SENSITIVITY;
  float gyroY = gy / GYRO_SENSITIVITY;
  float gyroZ = gz / GYRO_SENSITIVITY;
  
  // Compute roll and pitch from accelerometer in degrees
  float roll  = atan2(accelY, accelZ) * 180.0 / PI;
  float pitch = atan(-accelX / sqrt(accelY * accelY + accelZ * accelZ)) * 180.0 / PI;
  
  // Yaw calculation via integration of gyroZ 
  unsigned long currentGyroTime = millis();
  float dtGyro = (currentGyroTime - lastGyroTime) / 1000.0;
  lastGyroTime = currentGyroTime;
  yaw += gyroZ * dtGyro;  // simple integration
  
  // Servo control based on yaw 
  // Constrain yaw to the ±81° range for servo actuation
  float servoYaw = yaw;
  bool outOfRange = false;
  if (servoYaw > YAW_MAX) {
    servoYaw = YAW_MAX;
    outOfRange = true;
  } else if (servoYaw < YAW_MIN) {
    servoYaw = YAW_MIN;
    outOfRange = true;
  }
  
  // Map yaw from [-81, 81] to servo angle [0, 180]
  int servoAngle = map(servoYaw, YAW_MIN, YAW_MAX, 0, 180);
  servo.write(servoAngle);
  
  // LED (D3) brightness control based on X-axis acceleration 
  // If yaw is out-of-range, override by turning LED fully on
  if (outOfRange) {
    analogWrite(LED_PIN, 255);
  } else {
    // Use the absolute value of X-axis acceleration for brightness mapping
    float absAccelX = fabs(accelX);
    int brightness;
    if (absAccelX < ACC_THRESHOLD_LOW) {
      brightness = 0;
    } else if (absAccelX > ACC_THRESHOLD_HIGH) {
      brightness = 255;
    } else {
      // Linear mapping between thresholds
      brightness = (int)(((absAccelX - ACC_THRESHOLD_LOW) / (ACC_THRESHOLD_HIGH - ACC_THRESHOLD_LOW)) * 255);
    }
    analogWrite(LED_PIN, brightness);
  }
  
  // Distance calculation along X-axis 
  unsigned long currentDistanceTime = millis();
  float dtDistance = (currentDistanceTime - lastDistanceTime) / 1000.0;
  lastDistanceTime = currentDistanceTime;
  
  // Convert accelX (in g) to m/s^2 (1g ≈ 9.81 m/s^2)
  float accelX_mps2 = accelX * 9.81;
  velocityX += accelX_mps2 * dtDistance;
  distanceX += velocityX * dtDistance;
  
  if (millis() - lastOutputTime >= OUTPUT_INTERVAL) {
    lastOutputTime = millis();
    Serial.print("Roll: "); Serial.print(roll, 2);
    Serial.print("  Pitch: "); Serial.print(pitch, 2);
    Serial.print("  Yaw: "); Serial.print(yaw, 2);
    Serial.print("  AccX: "); Serial.print(accelX, 2);
    Serial.print("  AccY: "); Serial.print(accelY, 2);
    Serial.print("  AccZ: "); Serial.print(accelZ, 2);
    Serial.print("  Distance X: "); Serial.print(distanceX, 2);
    Serial.println(" m");
  }
}
