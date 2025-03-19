#include <Wire.h>
#include <Servo.h>  
#include "MPU6050.h"

/********************************************************************************
*********************************** MPU6050 *************************************
*********************************************************************************/

#define YAW_MAX 81   // Maximum allowed yaw in degrees
#define YAW_MIN -81  // Minimum allowed yaw in degrees
#define ACC_THRESHOLD_LOW  0.12   // Below this (g): LED off
#define ACC_THRESHOLD_HIGH 1.12    // Above this (g): LED fully on

// Sensor conversion factors 
#define ACCEL_SENSITIVITY 16384.0  // For ±2g mode (LSB/g)
#define GYRO_SENSITIVITY  131.0     // For ±250degree/sec mode (LSB/(degree/sec))

// Filter and integration constants 
const float compFilterAlpha = 0.98;  // Complementary filter weight for gyro
const float VELOCITY_THRESHOLD = 0.01;     // Deadband for velocity in m/s

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

// Accelerometer bias for X axis in g's
float accelBiasX = 0.0;

// Output interval (ms)
const unsigned long OUTPUT_INTERVAL = 1000;
unsigned long lastOutputTime = 0;
unsigned long currentTime;
unsigned long currentDistanceTime;
float dt;
float dtDistance;

// raw sensor data
int16_t ax, ay, az, gx, gy, gz;

// accleration and gyro readings
float ax_corr, ay_corr, az_corr;
float gyroX, gyroY, gyroZ;
float accRoll, accPitch;
float yaw_deg, servoYaw;

void xaxis_accelerometerCalibration();
void updateOrientation();

// sensor fusion variables
float g_sensor_x, g_sensor_y, g_sensor_z;
float ax_mps2, ay_mps2, az_mps2;
float a_world_x;
float ax_lin, ay_lin;
static float prevAccX = 0.0;
float newVelocityX;
const float ACCEL_DRIFT_THRESHOLD = 0.05;
static float stationaryTime = 0.0;

void sensorFusion();

/********************************************************************************
***********************************  SERVO  *************************************
*********************************************************************************/

#define SERVO_PIN 9    // Servo motor control (Controller P9)
#define LED_PIN 3      // LED "L" / D3 for brightness control

bool outOfRange;
int servoAngle;

void servoControl();

/********************************************************************************
***********************************  SETUP  *************************************
*********************************************************************************/

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

  xaxis_accelerometerCalibration();
}

/********************************************************************************
******************************  MAIN PROGRAM  *********************************** 
*********************************************************************************/

void loop() {
  updateOrientation();
  servoControl();
  sensorFusion();
}

/********************************************************************************
************************  FUNCTION IMPLEMENTATIONS  ***************************** 
*********************************************************************************/

void xaxis_accelerometerCalibration() {
  // Accelerometer Calibration (X-axis) 
  const int numSamples = 100;
  float sumX = 0.0;
  for (int i = 0; i < numSamples; i++) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    float rawAccelX = ax / ACCEL_SENSITIVITY;
    sumX += rawAccelX;
    delay(5);
  }
  accelBiasX = sumX / numSamples;
  Serial.print("Calibrated accelBiasX: ");
  Serial.println(accelBiasX, 4);
}

void updateOrientation() {
  // Orientation Update 
  currentTime = millis();
  dt = (currentTime - lastTimeOrientation) / 1000.0;
  if (dt <= 0) dt = 0.01;
  lastTimeOrientation = currentTime;

  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Correct accelerometer readings (in g's)
  ax_corr = (ax / ACCEL_SENSITIVITY) - accelBiasX;
  ay_corr = (ay / ACCEL_SENSITIVITY);
  az_corr = (az / ACCEL_SENSITIVITY);

  // Gyro readings in deg/s, convert to rad/s
  gyroX = (gx / GYRO_SENSITIVITY) * DEG_TO_RAD;
  gyroY = (gy / GYRO_SENSITIVITY) * DEG_TO_RAD;
  gyroZ = (gz / GYRO_SENSITIVITY) * DEG_TO_RAD;

  // Compute accelerometer-based angles (in radians)
  accRoll = atan2(ay_corr, az_corr);
  accPitch = atan2(-ax_corr, sqrt(ay_corr * ay_corr + az_corr * az_corr));

  // Complementary filter for roll and pitch, integrate gyro for yaw
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
}

void sensorFusion() {
  // Sensor Fusion for Displacement 
  // Convert corrected accelerometer readings to m/s²
  ax_mps2 = ax_corr * 9.81;
  ay_mps2 = ay_corr * 9.81;
  az_mps2 = az_corr * 9.81;

  // Compute expected gravity vector in sensor frame from fused roll and pitch
  g_sensor_x = -sin(fusedPitch) * 9.81;
  g_sensor_y = sin(fusedRoll) * cos(fusedPitch) * 9.81;
  g_sensor_z = cos(fusedRoll) * cos(fusedPitch) * 9.81;

  // Subtract gravity to obtain linear acceleration (sensor frame)
  ax_lin = ax_mps2 - g_sensor_x;
  ay_lin = ay_mps2 - g_sensor_y;
  // We'll use only the horizontal components for displacement

  // Rotate horizontal (x-y) linear acceleration by fused yaw to align with world X–axis
  a_world_x = ax_lin * cos(fusedYaw) - ay_lin * sin(fusedYaw);

  // Distance Integration (Trapezoidal with Zero Velocity Update) 
  currentDistanceTime = millis();
  dtDistance = (currentDistanceTime - lastDistanceTime) / 1000.0;
  lastDistanceTime = currentDistanceTime;

  // Integrate a_world_x to update velocity using trapezoidal rule
  newVelocityX = velocityX + 0.5 * (prevAccX + a_world_x) * dtDistance;
  if (fabs(newVelocityX) < VELOCITY_THRESHOLD)
      newVelocityX = 0.0;

  // Zero velocity update: if acceleration remains near zero for a period, reset velocity
  stationaryTime = 0.0;
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

  // Output Data Once Per Second 
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

void servoControl() {
  // Servo Control Based on Yaw 
  yaw_deg = fusedYaw * RAD_TO_DEG;
  servoYaw = yaw_deg;
  outOfRange = false;
  if (servoYaw > YAW_MAX) {
    servoYaw = YAW_MAX;
    outOfRange = true;
  } else if (servoYaw < YAW_MIN) {
    servoYaw = YAW_MIN;
    outOfRange = true;
  }
  servoAngle = map(servoYaw, YAW_MIN, YAW_MAX, 0, 180);
  servo.write(servoAngle);
}
