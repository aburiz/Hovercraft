#include <Wire.h>
#include <Servo.h>
#include <MPU6050.h>

#define DEBUG 1  // Set to 1 for debugging prints

// ----- Create objects -----
MPU6050 mpu;
Servo thrustServo;

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
const float DRIFT_CORRECTION_GAIN = 2.0;  // Adjust to scale correction (can be tuned)

// Timing variables for turn integration
unsigned long lastTurnTime = 0;  // Enforce a cooldown between turns
unsigned long prevTurnTime = 0;  // For integration during a turn

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
// This function sweeps the servo from 0° to 180° in steps and returns the angle
// where the ultrasonic sensor measures the maximum distance (i.e. the best open path).
void scanForOpenPath(int &chosenAngle) {
  const int step = 15;  // Adjust step size for resolution versus speed.
  int bestAngle = SERVO_NEUTRAL;
  long bestDistance = 0;
  
  for (int angle = 0; angle <= 180; angle += step) {
    thrustServo.write(angle);
    delay(50);  // Allow servo to reach position.
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
  
  // Return servo to neutral
  thrustServo.write(SERVO_NEUTRAL);
  delay(50);
  
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
  // Here, we use SERVO_RIGHT if turning right (direction > 0) and SERVO_LEFT for left.
  thrustServo.write((direction > 0) ? SERVO_RIGHT : SERVO_LEFT);
  delay(50);
  
  // Turn off thrust during the turn.
  digitalWrite(THRUST_FAN_PIN, LOW);
  
  unsigned long turnStart = millis();
  while (abs(heading) < TARGET_TURN_ANGLE && (millis() - turnStart < TURN_TIMEOUT_MS)) {
    unsigned long now = millis();
    float dt = (now - prevTurnTime) / 1000.0;
    prevTurnTime = now;
    
    int16_t gx, gy, gz;
    mpu.getRotation(&gx, &gy, &gz);
    float angularRate = gz / 131.0;  // Convert raw value to degrees per second.
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
  
  // Reset drift correction variables after the turn.
  heading = 0.0;
  lastDriftTime = millis();
  lastTurnTime = millis();
}

// ----- Drift Correction Using Full Servo Range -----
// In normal forward motion, the servo angle is computed as SERVO_NEUTRAL plus a correction
// derived from the integrated yaw error. The output is clamped between 0 and 180.
void updateDriftCorrection() {
  unsigned long now = millis();
  float dt = (now - lastDriftTime) / 1000.0;
  lastDriftTime = now;
  
  int16_t gx, gy, gz;
  mpu.getRotation(&gx, &gy, &gz);
  float rawYawRate = gz / 131.0;  // degrees per second
  float filteredYawRate = 0.9 * rawYawRate + 0.1 * lastYawRate;
  lastYawRate = filteredYawRate;
  
  // Integrate with decay to avoid runaway accumulation.
  heading = heading * 0.99 + filteredYawRate * dt;
  
  // Compute correction (target heading is 0).
  float error = heading;
  float correction = error * DRIFT_CORRECTION_GAIN;
  
  // Compute new servo angle using full range.
  int correctedAngle = SERVO_NEUTRAL + (int)correction;
  
  // Clamp the servo angle between 0 and 180.
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
  delay(1000);  // Allow sensors and IMU to stabilize.
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
      
      // Decide turn direction: if chosen angle > neutral, open is to the right; otherwise, left.
      int turnDir = (chosenAngle > SERVO_NEUTRAL) ? 1 : -1;
      turnUsingIMU(turnDir);
    }
  }
  
  // ----- Normal Forward Movement with Drift Correction -----
  digitalWrite(THRUST_FAN_PIN, HIGH);
  updateDriftCorrection();
  
  delay(50);
}
