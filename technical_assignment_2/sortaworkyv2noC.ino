#include <Wire.h>
#include <Servo.h>
#include "MPU6050.h"

#define SERVO_PIN 9    // Servo motor control (Controller P9)
#define LED_PIN 3      // LED "L" / D3 for brightness control

#define YAW_MAX 81   // Maximum allowed yaw (degrees)
#define YAW_MIN -81  // Minimum allowed yaw (degrees)
#define ACC_THRESHOLD_LOW  0.12   // Below this (g): LED off
#define ACC_THRESHOLD_HIGH 1.12    // Above this (g): LED fully on

// --- Sensor conversion factors ---
#define ACCEL_SENSITIVITY 16384.0  // For ±2g mode (LSB/g)
#define GYRO_SENSITIVITY  131.0     // For ±250°/sec mode (LSB/(°/sec))

// --- Filter and integration constants ---
const float compFilterAlpha = 0.98;  // Complementary filter weight for gyro
const float VELOCITY_THRESHOLD = 0.01;     // Deadband for velocity (m/s)

MPU6050 mpu;
Servo servo;

// Orientation (in radians)
float fusedRoll = 0.0, fusedPitch = 0.0, fusedYaw = 0.0;
bool firstOrientation = true;
unsigned long lastTimeOrientation = 0;

// Displacement integration variables (world X axis)
float distanceX = 0.0;
float velocityX = 0.0;
unsigned long lastDistanceTime = 0;

// Accelerometer bias for X axis (in g's)
float accelBiasX = 0.0;

// Output interval (ms)
const unsigned long OUTPUT_INTERVAL = 1000;
unsigned long lastOutputTime = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1);
  }

  // Set sensor full-scale ranges
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);

  servo.attach(SERVO_PIN);
  pinMode(LED_PIN, OUTPUT);

  lastTimeOrientation = millis();
  lastDistanceTime = millis();
  lastOutputTime = millis();

  // --- Accelerometer Calibration (X-axis) ---
  const int numSamples = 100;
  float sumX = 0.0;
  for (int i = 0; i < numSamples; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    float rawAccelX = ax / ACCEL_SENSITIVITY;
    sumX += rawAccelX;
    delay(5);
  }
  accelBiasX = sumX / numSamples;
  Serial.print("Calibrated accelBiasX: ");
  Serial.println(accelBiasX, 4);
}

void loop() {
  // --- Orientation Update ---
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTimeOrientation) / 1000.0;
  if (dt <= 0) dt = 0.01;
  lastTimeOrientation = currentTime;

  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Correct accelerometer readings (in g's)
  float ax_corr = (ax / ACCEL_SENSITIVITY) - accelBiasX;
  float ay_corr = (ay / ACCEL_SENSITIVITY);
  float az_corr = (az / ACCEL_SENSITIVITY);

  // Gyro readings in deg/s, convert to rad/s
  float gyroX = (gx / GYRO_SENSITIVITY) * DEG_TO_RAD;
  float gyroY = (gy / GYRO_SENSITIVITY) * DEG_TO_RAD;
  float gyroZ = (gz / GYRO_SENSITIVITY) * DEG_TO_RAD;

  // Compute accelerometer-based angles (in radians)
  float accRoll = atan2(ay_corr, az_corr);
  float accPitch = atan2(-ax_corr, sqrt(ay_corr * ay_corr + az_corr * az_corr));

  // Complementary filter for roll and pitch; integrate gyro for yaw
  if (firstOrientation) {
    fusedRoll = accRoll;
    fusedPitch = accPitch;
    fusedYaw = 0.0;
    firstOrientation = false;
  } else {
    fusedRoll = compFilterAlpha * (fusedRoll + gyroX * dt) + (1 - compFilterAlpha) * accRoll;
    fusedPitch = compFilterAlpha * (fusedPitch + gyroY * dt) + (1 - compFilterAlpha) * accPitch;
    fusedYaw = fusedYaw + gyroZ * dt;  // Yaw from gyro integration only
  }

  // --- Servo Control Based on Yaw ---
  float yaw_deg = fusedYaw * RAD_TO_DEG;
  float servoYaw = yaw_deg;
  bool outOfRange = false;
  if (servoYaw > YAW_MAX) {
    servoYaw = YAW_MAX;
    outOfRange = true;
  } else if (servoYaw < YAW_MIN) {
    servoYaw = YAW_MIN;
    outOfRange = true;
  }
  int servoAngle = map(servoYaw, YAW_MIN, YAW_MAX, 0, 180);
  servo.write(servoAngle);

  // --- LED Brightness Control (using raw X-axis acceleration) ---
  if (outOfRange) {
    analogWrite(LED_PIN, 255);
  } else {
    float absAccelX = fabs(ax_corr);
    int brightness;
    if (absAccelX < ACC_THRESHOLD_LOW)
      brightness = 0;
    else if (absAccelX > ACC_THRESHOLD_HIGH)
      brightness = 255;
    else
      brightness = (int)(((absAccelX - ACC_THRESHOLD_LOW) / (ACC_THRESHOLD_HIGH - ACC_THRESHOLD_LOW)) * 255);
    analogWrite(LED_PIN, brightness);
  }

  // --- Sensor Fusion for Displacement ---
  // Convert corrected accelerometer readings to m/s²
  float ax_mps2 = ax_corr * 9.81;
  float ay_mps2 = ay_corr * 9.81;
  float az_mps2 = az_corr * 9.81;

  // Compute expected gravity vector in sensor frame from fused roll and pitch
  float g_sensor_x = -sin(fusedPitch) * 9.81;
  float g_sensor_y = sin(fusedRoll) * cos(fusedPitch) * 9.81;
  float g_sensor_z = cos(fusedRoll) * cos(fusedPitch) * 9.81;

  // Subtract gravity to obtain linear acceleration (sensor frame)
  float ax_lin = ax_mps2 - g_sensor_x;
  float ay_lin = ay_mps2 - g_sensor_y;
  // We'll use only the horizontal components for displacement

  // Rotate horizontal (x-y) linear acceleration by fused yaw to align with world X–axis
  float a_world_x = ax_lin * cos(fusedYaw) - ay_lin * sin(fusedYaw);

  // --- Distance Integration (Trapezoidal with Zero Velocity Update) ---
  unsigned long currentDistanceTime = millis();
  float dtDistance = (currentDistanceTime - lastDistanceTime) / 1000.0;
  lastDistanceTime = currentDistanceTime;

  // Integrate a_world_x to update velocity using trapezoidal rule
  static float prevAccX = 0.0;
  float newVelocityX = velocityX + 0.5 * (prevAccX + a_world_x) * dtDistance;
  if (fabs(newVelocityX) < VELOCITY_THRESHOLD)
      newVelocityX = 0.0;

  // Zero velocity update: if acceleration remains near zero for a period, reset velocity
  const float ACCEL_DRIFT_THRESHOLD = 0.05; // m/s² threshold for stationary detection
  static float stationaryTime = 0.0;
  if (fabs(a_world_x) < ACCEL_DRIFT_THRESHOLD)
    stationaryTime += dtDistance;
  else
    stationaryTime = 0.0;
  if (stationaryTime > 0.2) {  // if stationary for more than 0.2 seconds
    newVelocityX = 0.0;
  }

  // Update displacement using trapezoidal integration of velocity
  distanceX += 0.5 * (velocityX + newVelocityX) * dtDistance;
  velocityX = newVelocityX;
  prevAccX = a_world_x;

  // --- Output Data Once Per Second ---
  if (millis() - lastOutputTime >= OUTPUT_INTERVAL) {
    lastOutputTime = millis();
    Serial.print("Roll: "); Serial.print(fusedRoll * RAD_TO_DEG, 2);
    Serial.print("  Pitch: "); Serial.print(fusedPitch * RAD_TO_DEG, 2);
    Serial.print("  Yaw: "); Serial.print(yaw_deg, 2);
    Serial.print("  AccX: "); Serial.print(ax_corr, 2);
    Serial.print("  AccY: "); Serial.print(ay_corr, 2);
    Serial.print("  AccZ: "); Serial.print(az_corr, 2);
    Serial.print("  Distance X: "); Serial.print(distanceX, 4);
    Serial.println(" m");
  }
}
