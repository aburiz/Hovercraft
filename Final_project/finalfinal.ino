#include <Wire.h>
#include <Servo.h>
#include <MPU6050.h>

// ==================
//   PID SETTINGS
// ==================
float Kp = 2.5;  // Proportional gain
float Ki = 0.0;  // Integral gain
float Kd = 1.0;  // Derivative gain

float headingErrorIntegral = 0.0;  
float lastHeadingError     = 0.0;  
const float INTEGRAL_CLAMP = 20.0; 

// ==================
//   MPU & SERVO
// ==================
MPU6050 mpu;
Servo thrustServo;
float gyroZBias = 0.0;  // Global calibration for the gyro Z bias

// ----------------------
//     Pin Definitions
// ----------------------
// NOTE: If you're using an Arduino Uno, digital pin 13 is PB5, and digital pin 11 is PB3.
const uint8_t FRONT_ECHO_PIN = 3;  // D3 (Echo)
const uint8_t UP_ECHO_PIN    = 2;  // D2 (Echo)

// Fan Control Pins
const uint8_t THRUST_FAN_PIN = 6;  // PWM-capable
const uint8_t LIFT_FAN_PIN   = 7;  // Simple on/off

// Servo Control Pin
const uint8_t SERVO_PIN = 9;       

// Thrust Fan Speed Settings
const int DEFAULT_THRUST_FAN_SPEED = 200;

// Separate LEFT / RIGHT Turn Speeds
const int LEFT_TURN_FAN_SPEED  = 200;  
const int RIGHT_TURN_FAN_SPEED = 160;  

// Distance Thresholds
const int WALL_THRESHOLD = 20;  
const int BAR_THRESHOLD  = 45;  

// Servo Angles
const int SERVO_NEUTRAL = 93;   
#define SERVO_LEFT   25
#define SERVO_RIGHT  105

// Turn Config
const float TARGET_TURN_ANGLE     = 180.0;   
const unsigned long TURN_TIMEOUT_MS   = 4000;
const unsigned long TURN_COOLDOWN_MS  = 1000;
const unsigned long TURN_HOLD_DELAY_MS = 0;

// Heading / Yaw Integration
float heading         = 0.0;      
unsigned long lastDriftTime = 0;  
unsigned long lastTurnTime  = 0;  
unsigned long prevTurnTime  = 0;  

// Low-pass filter memory
float lastYawRate = 0.0;

// For debug prints (set to 0 to disable)
#define DEBUG 1

// --------------
//  Calibration
// --------------
float calibrateGyroZ() {
  const int samples = 500;
  float sum = 0.0;
  for (int i = 0; i < samples; i++) {
    int16_t gx, gy, gz;
    mpu.getRotation(&gx, &gy, &gz);
    float rate = gz / 131.0; // raw -> deg/sec
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

// -------------
//  Triggers
// -------------
// Front sensor: TRIG on D13 (PORTB bit5)
inline void triggerPulseFront() {
  PORTB &= ~(1 << PB5);
  delayMicroseconds(2);
  PORTB |=  (1 << PB5);
  delayMicroseconds(10);
  PORTB &= ~(1 << PB5);
}

// Upward sensor: TRIG on D11 (PORTB bit3)
inline void triggerPulseUp() {
  PORTB &= ~(1 << PB3);
  delayMicroseconds(2);
  PORTB |=  (1 << PB3);
  delayMicroseconds(10);
  PORTB &= ~(1 << PB3);
}

// ------------------------
//  Ultrasonic Distance
// ------------------------
long measureDistanceFront() {
  triggerPulseFront();
  long duration = pulseIn(FRONT_ECHO_PIN, HIGH, 30000); // 30ms timeout
  return duration * 0.034 / 2;
}

long measureDistanceUp() {
  triggerPulseUp();
  long duration = pulseIn(UP_ECHO_PIN, HIGH, 30000); // 30ms timeout
  return duration * 0.034 / 2;
}

// ---------------------
//  Full-Range Scanning
// ---------------------
void scanForOpenPath(int &chosenAngle) {
  const int step = 15;
  int bestAngle   = SERVO_NEUTRAL;
  long bestDistance = 0;

  for (int angle = 0; angle <= 180; angle += step) {
    thrustServo.write(angle);
    delay(150);
    long distance = measureDistanceFront();
#if DEBUG
    Serial.print("Angle: ");
    Serial.print(angle);
    Serial.print(" -> Distance: ");
    Serial.println(distance);
#endif
    if (distance > bestDistance) {
      bestDistance = distance;
      bestAngle = angle;
    }
  }

  thrustServo.write(SERVO_NEUTRAL);
  delay(20);
  chosenAngle = bestAngle;
#if DEBUG
  Serial.print("Chosen angle: ");
  Serial.println(chosenAngle);
#endif
}

// ---------------------
//  Turn Integration
// ---------------------
void resetTurnIntegration() {
  prevTurnTime = millis();
  heading      = 0.0;
}

void turnUsingIMU(int direction) {
#if DEBUG
  Serial.println("Starting turn...");
#endif
  resetTurnIntegration();

  // Position servo for turn
  thrustServo.write((direction > 0) ? SERVO_RIGHT : SERVO_LEFT);
  delay(50);

  // Use distinct fan speeds for LEFT vs. RIGHT
  if (direction > 0) {
    // Right turn => use right turn fan speed
    analogWrite(THRUST_FAN_PIN, RIGHT_TURN_FAN_SPEED);
  } else {
    // Left turn => use left turn fan speed
    analogWrite(THRUST_FAN_PIN, LEFT_TURN_FAN_SPEED);
  }

  unsigned long turnStart = millis();
  while (abs(heading) < TARGET_TURN_ANGLE &&
         (millis() - turnStart < TURN_TIMEOUT_MS)) {
    unsigned long now = millis();
    float dt = (now - prevTurnTime) / 1000.0;
    prevTurnTime = now;

    int16_t gx, gy, gz;
    mpu.getRotation(&gx, &gy, &gz);
    float angularRate = (gz / 131.0) - gyroZBias;
    heading += angularRate * dt;

    delay(5);
  }

  // Extra hold
  delay(TURN_HOLD_DELAY_MS);

#if DEBUG
  Serial.println("Turn complete.");
#endif

  // Reset servo to neutral
  thrustServo.write(SERVO_NEUTRAL);
  delay(50);

  // Restore thrust fan speed
  analogWrite(THRUST_FAN_PIN, DEFAULT_THRUST_FAN_SPEED);

  // Reset heading and timers
  heading               = 0.0;
  lastDriftTime         = millis();
  lastTurnTime          = millis();
  headingErrorIntegral  = 0.0;
  lastHeadingError      = 0.0;
}

// -------------------
//   PID Correction
// -------------------
void updateDriftCorrection() {
  unsigned long now = millis();
  float dt = (now - lastDriftTime) / 1000.0;
  lastDriftTime = now;

  // Get gyro reading
  int16_t gx, gy, gz;
  mpu.getRotation(&gx, &gy, &gz);
  
  // Low-pass filter the gyro reading
  float rawYawRate   = (gz / 131.0) - gyroZBias;
  float filteredRate = 0.9 * rawYawRate + 0.1 * lastYawRate;
  lastYawRate        = filteredRate;

  // Integrate heading from the filtered gyro rate
  heading += filteredRate * dt;

  // We want heading = 0, so:
  float headingError = -heading;

  // ============ PID Computation ============
  headingErrorIntegral += headingError * dt;
  if (headingErrorIntegral > INTEGRAL_CLAMP) headingErrorIntegral = INTEGRAL_CLAMP;
  if (headingErrorIntegral < -INTEGRAL_CLAMP) headingErrorIntegral = -INTEGRAL_CLAMP;

  float headingErrorDerivative = (headingError - lastHeadingError) / dt;
  float pidOutput = (Kp * headingError)
                  + (Ki * headingErrorIntegral)
                  + (Kd * headingErrorDerivative);
  lastHeadingError = headingError;
  // ========================================

  // Translate PID output to servo angle
  int correctedAngle = SERVO_NEUTRAL + (int)pidOutput;

  // Clamp servo angle
  if (correctedAngle < 0)   correctedAngle = 0;
  if (correctedAngle > 180) correctedAngle = 180;

  thrustServo.write(correctedAngle);

#if DEBUG
  Serial.print("Heading: ");
  Serial.print(heading);
  Serial.print(" | Error: ");
  Serial.print(headingError);
  Serial.print(" | PID Output: ");
  Serial.print(pidOutput);
  Serial.print(" | Servo Angle: ");
  Serial.println(correctedAngle);
#endif
}

void setup() {
#if DEBUG
  Serial.begin(9600);
#endif
  Wire.begin();
  mpu.initialize();

  // -------------------------------------------------
  // 1) Make sure TRIG PINS are set as OUTPUT
  //    for both front (D13) and top (D11) triggers
  // -------------------------------------------------
  pinMode(13, OUTPUT);  // Trigger for front sensor
  pinMode(11, OUTPUT);  // Trigger for upward sensor

  // ECHO pins as input
  pinMode(FRONT_ECHO_PIN, INPUT); // D3
  pinMode(UP_ECHO_PIN, INPUT);    // D2

  // Fan pins
  pinMode(THRUST_FAN_PIN, OUTPUT);
  pinMode(LIFT_FAN_PIN, OUTPUT);

  // Attach servo
  thrustServo.attach(SERVO_PIN);
  thrustServo.write(SERVO_NEUTRAL);

  // -------------------------------------------------
  //   Calibration & Startup
  // -------------------------------------------------
  delay(1000);
  gyroZBias = calibrateGyroZ();

  analogWrite(THRUST_FAN_PIN, DEFAULT_THRUST_FAN_SPEED);
  digitalWrite(LIFT_FAN_PIN, HIGH);

  lastDriftTime = millis();
  lastTurnTime  = millis();
  delay(1000);
}

void loop() {
  // -------------------------------------------------
  // 2) Print the "up" distance to confirm reading
  // -------------------------------------------------
  long barDistance = measureDistanceUp();
#if DEBUG
  Serial.print("Bar Distance (Up): ");
  Serial.println(barDistance);
#endif

  // If barDistance < BAR_THRESHOLD => overhead obstacle
  if (barDistance > 0 && barDistance < BAR_THRESHOLD) {
#if DEBUG
    Serial.println("Bar detected! Stopping.");
#endif
    analogWrite(THRUST_FAN_PIN, 0);
    digitalWrite(LIFT_FAN_PIN, LOW);
    while (true) {
      delay(100);
    }
  }

  // =============
  //  Wall Check
  // =============
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
      // Turn off fans during scan
      analogWrite(THRUST_FAN_PIN, 0);
      digitalWrite(LIFT_FAN_PIN, LOW);

      int chosenAngle = SERVO_NEUTRAL;
      scanForOpenPath(chosenAngle);

      // Decide direction
      int turnDir = (chosenAngle > SERVO_NEUTRAL) ? 1 : -1;

      // Turn fans on again (but at turn speed as chosen in turnUsingIMU)
      if (turnDir > 0)
        analogWrite(THRUST_FAN_PIN, RIGHT_TURN_FAN_SPEED);
      else
        analogWrite(THRUST_FAN_PIN, LEFT_TURN_FAN_SPEED);
      digitalWrite(LIFT_FAN_PIN, HIGH);

      turnUsingIMU(turnDir);
    }
  }

  // =============
  //  Normal Drive
  // =============
  analogWrite(THRUST_FAN_PIN, DEFAULT_THRUST_FAN_SPEED);

  // Use the PID-based drift correction
  updateDriftCorrection();

  delay(50);
}
