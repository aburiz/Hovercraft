#include <Wire.h>
#include <Servo.h>
#include <MPU6050.h>

#define DEBUG 1  // Set to 1 for debugging prints, 0 to disable

// ----- Create objects -----
MPU6050 mpu;
Servo thrustServo;

// ----- Global Calibration Variable -----
float gyroZBias = 0.0;  // Bias for the Z-axis gyro

// ----- Pin Definitions -----
// Front sensor: TRIG on D13 (PORTB, bit 5), ECHO on D3
const uint8_t FRONT_ECHO_PIN = 3;   // D3

// Upward sensor: TRIG on D11 (PORTB, bit 3), ECHO on D2
const uint8_t UP_ECHO_PIN = 2;      // D2

// Fan control pins
const uint8_t THRUST_FAN_PIN = 4;   // D4
const uint8_t LIFT_FAN_PIN   = 7;   // D7

// Servo control pin (HS‑422)
const uint8_t SERVO_PIN = 9;        // D9

// ----- Configuration -----
// Distances in cm
const int WALL_THRESHOLD = 15;     // If front sensor reads less than 15 cm, a wall is detected.
const int BAR_THRESHOLD  = 15;     // If upward sensor reads less than 15 cm, the bar is detected.

// Servo angles (using full 0-180 for maximum turning force)
const int SERVO_NEUTRAL = 90;      // Neutral position
#define SERVO_LEFT   0           // Left extreme
#define SERVO_RIGHT  180         // Right extreme

// Turning settings (for IMU-assisted turns)
const float TARGET_TURN_ANGLE = 90.0;         // degrees for a turn
const unsigned long TURN_TIMEOUT_MS = 2000;   // Maximum allowed time for a turn
const unsigned long TURN_COOLDOWN_MS = 500;     // Cooldown between turns

// ----- Drift Correction Variables -----
float heading = 0.0;             // Integrated yaw drift (degrees)
unsigned long lastDriftTime = 0; // Timestamp for drift integration
float lastYawRate = 0.0;         // For low-pass filtering the gyro reading
// Increase or decrease gain so that full drift drives the servo nearly to 0 or 180.
const float DRIFT_CORRECTION_GAIN = 2.0;  // Adjust to scale correction

// Timing variables for turn integration
unsigned long lastTurnTime = 0;  // Enforce a cooldown between turns
unsigned long prevTurnTime = 0;  // For integration during a turn

// ----- Function: Calibrate the MPU6050 Gyro Z Bias -----
// Samples the gyro when the craft is stationary and averages the readings.
float calibrateGyroZ() {
  const int samples = 500;
  float sum = 0.0;
  for (int i = 0; i < samples; i++) {
    int16_t gx, gy, gz;
    mpu.getRotation(&gx, &gy, &gz);
    // Convert raw Z-axis reading to degrees per second.
    float rate = gz / 131.0;
    sum += rate;
    delay(5);
  }
  float bias = sum / samples;
#if DEBUG
  Serial.print("Gyro Z Bias: ");
  Serial.println(bias);
#endif
  return bias;
}

// ----- Inline functions for fast ultrasonic triggers -----
// For front sensor: TRIG on D13 (PORTB, bit 5)
inline void triggerPulseFront() {
  PORTB &= ~(1 << PB5);   // Clear bit PB5 (D13)
  delayMicroseconds(2);
  PORTB |= (1 << PB5);    // Set bit PB5
  delayMicroseconds(10);
  PORTB &= ~(1 << PB5);
}

// For upward sensor: TRIG on D11 (PORTB, bit 3)
inline void triggerPulseUp() {
  PORTB &= ~(1 << PB3);   // Clear bit PB3 (D11)
  delayMicroseconds(2);
  PORTB |= (1 << PB3);    // Set bit PB3
  delayMicroseconds(10);
  PORTB &= ~(1 << PB3);
}

// ----- Distance Measurement -----
long measureDistanceFront() {
  triggerPulseFront();
  long duration = pulseIn(FRONT_ECHO_PIN, HIGH, 30000); // Timeout of 30ms
  return duration * 0.034 / 2;  // Convert microseconds to centimeters.
}

long measureDistanceUp() {
  triggerPulseUp();
  long duration = pulseIn(UP_ECHO_PIN, HIGH, 30000);
  return duration * 0.034 / 2;
}

// ----- Full-Range Path Scanning -----
// Sweeps the servo from 0° to 180° in steps and returns the angle with the best opening.
void scanForOpenPath(int &chosenAngle) {
  const int step = 15;  // Step size in degrees.
  int bestAngle = SERVO_NEUTRAL;
  long bestDistance = 0;
  
  for (int angle = 0; angle <= 180; angle += step) {
    thrustServo.write(angle);
    delay(50);  // Reduced delay per step for faster scanning.
    long distance = measureDistanceFront();
#if DEBUG
    Serial.print("Angle: "); Serial.print(angle);
    Serial.print(" -> Distance: "); Serial.println(distance);
#endif
    if (distance > bestDistance) {
      bestDistance = distance;
      bestAngle = angle;
    }
  }
  
  thrustServo.write(SERVO_NEUTRAL);
  delay(20);  // Reduced delay after scanning.
  chosenAngle = bestAngle;
#if DEBUG
  Serial.print("Chosen angle: "); Serial.println(chosenAngle);
#endif
}

// ----- Turn Integration -----
void resetTurnIntegration() {
  prevTurnTime = millis();
  heading = 0.0;
}

void turnUsingIMU(int direction) {
#if DEBUG
  Serial.println("Starting turn...");
#endif
  resetTurnIntegration();
  
  // Set servo toward the desired turn direction.
  thrustServo.write((direction > 0) ? SERVO_RIGHT : SERVO_LEFT);
  delay(50);
  
  // Stop the thrust fan during the turn.
  digitalWrite(THRUST_FAN_PIN, LOW);
  
  unsigned long turnStart = millis();
  while (abs(heading) < TARGET_TURN_ANGLE && (millis() - turnStart < TURN_TIMEOUT_MS)) {
    unsigned long now = millis();
    float dt = (now - prevTurnTime) / 1000.0;
    prevTurnTime = now;
    
    int16_t gx, gy, gz;
    mpu.getRotation(&gx, &gy, &gz);
    // Subtract the bias from the gyro reading.
    float angularRate = (gz / 131.0) - gyroZBias;
    heading += angularRate * dt;
    
    delay(5);
  }
#if DEBUG
  Serial.println("Turn complete.");
#endif
  
  // Reset servo to neutral and restart thrust.
  thrustServo.write(SERVO_NEUTRAL);
  delay(50);
  digitalWrite(THRUST_FAN_PIN, HIGH);
  
  // Reset drift variables after the turn.
  heading = 0.0;
  lastDriftTime = millis();
  lastTurnTime = millis();
}

// ----- Drift Correction Using Full Servo Range -----
// Reads the gyro, applies bias correction, and adjusts the servo from 0 to 180 accordingly.
void updateDriftCorrection() {
  unsigned long now = millis();
  float dt = (now - lastDriftTime) / 1000.0;
  lastDriftTime = now;
  
  int16_t gx, gy, gz;
  mpu.getRotation(&gx, &gy, &gz);
  float rawYawRate = (gz / 131.0) - gyroZBias;  // Subtract bias here.
  float filteredYawRate = 0.9 * rawYawRate + 0.1 * lastYawRate;
  lastYawRate = filteredYawRate;
  
  // Integrate with decay to avoid runaway accumulation.
  heading = heading * 0.99 + filteredYawRate * dt;
  
  // Compute correction (aiming for heading of 0).
  float error = -heading;
  float correction = error * DRIFT_CORRECTION_GAIN;
  
  // Compute the new servo angle.
  int correctedAngle = SERVO_NEUTRAL + (int)correction;
  
  // Clamp the servo angle to [0, 180].
  if (correctedAngle < 0)   correctedAngle = 0;
  if (correctedAngle > 180) correctedAngle = 180;
  
  thrustServo.write(correctedAngle);
  
#if DEBUG
  Serial.print("Heading: "); Serial.print(heading);
  Serial.print(" | Correction: "); Serial.print(correction);
  Serial.print(" | Servo Angle: "); Serial.println(correctedAngle);
#endif
}

void setup() {
#if DEBUG
  Serial.begin(9600);
#endif
  Wire.begin();
  mpu.initialize();
  
  // Allow the MPU6050 time to stabilize.
  delay(1000);
  // Calibrate the gyro (make sure the craft is stationary during calibration).
  gyroZBias = calibrateGyroZ();
  
  thrustServo.attach(SERVO_PIN);
  thrustServo.write(SERVO_NEUTRAL);
  
  // Configure sensor echo pins.
  pinMode(FRONT_ECHO_PIN, INPUT);
  pinMode(UP_ECHO_PIN, INPUT);
  
  // Configure fan control pins.
  pinMode(THRUST_FAN_PIN, OUTPUT);
  pinMode(LIFT_FAN_PIN, OUTPUT);
  
  // Start fans.
  digitalWrite(LIFT_FAN_PIN, HIGH);
  digitalWrite(THRUST_FAN_PIN, HIGH);
  
  lastDriftTime = millis();
  lastTurnTime  = millis();
  delay(1000);  // Allow sensors and IMU to stabilize further.
}

void loop() {
  // ----- Overhead Bar Detection -----
  long barDistance = measureDistanceUp();
  if (barDistance > 0 && barDistance < BAR_THRESHOLD) {
#if DEBUG
    Serial.println("Bar detected! Stopping.");
#endif
    digitalWrite(THRUST_FAN_PIN, LOW);
    while (true) { delay(100); }
  }
  
  // ----- Wall Detection & Turning -----
  // Only check if cooldown period has passed.
  if (millis() - lastTurnTime > TURN_COOLDOWN_MS) {
    long frontDistance = measureDistanceFront();
#if DEBUG
    Serial.print("Front Distance: ");
    Serial.println(frontDistance);
#endif
    if (frontDistance > 0 && frontDistance < WALL_THRESHOLD) {
#if DEBUG
      Serial.println("Wall detected! Scanning for open path...");
#endif
      int chosenAngle = SERVO_NEUTRAL;
      scanForOpenPath(chosenAngle);
      
      // Decide turn direction: if chosen angle > neutral, then open is to the right.
      int turnDir = (chosenAngle > SERVO_NEUTRAL) ? 1 : -1;
      turnUsingIMU(turnDir);
    }
  }
  
  // ----- Normal Forward Movement with Drift Correction -----
  digitalWrite(THRUST_FAN_PIN, HIGH);
  updateDriftCorrection();
  
  delay(50);
}