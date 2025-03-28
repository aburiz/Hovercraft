// Improved navigation system with better PID implementation
// Based on the winning implementation that achieved a 27-second time

#include <Wire.h>
#include <Servo.h>
#include <mpu6050.h>  // Changed to use the mpu6050 library in the workspace
#include <HCSR04.h>   // Using the HCSR04 library for distance measurement

// Debug flag for serial output
#define DEBUG 1

// ----- Create objects -----
MPU6050 mpu6050;  // Changed to match the workspace library
Servo thrustServo;
HCSR04 frontSensor(13, 3);  // TRIG pin on D13, ECHO pin on D3

// Pin Definitions
const uint8_t THRUST_FAN_PIN = 4;   // D4
const uint8_t LIFT_FAN_PIN   = 7;   // D7
const uint8_t SERVO_PIN = 9;        // D9

// Configuration
const int WALL_THRESHOLD = 40;      // Increased from 15cm to 40cm like winning implementation
const int EXIT_THRESHOLD = 170;     // Distance threshold for detecting the exit
const int SERVO_NEUTRAL = 90;
const int RIGHT_ANGLE = 180;
const int LEFT_ANGLE = 0;

// Turn patterns
bool right = true;
bool left = false;
int turnCount = 0;
int exitTurnCount = 3;  // After this many turns, consider looking for exit

// PID constants - taken from winning implementation
#define PID_K_p 0.5
#define PID_K_i 0.1
#define PID_K_d 2.0

// PID variables
float i_input = 0;
float d_last = 0;
float targetYaw = 0;

// Function to initialize MPU6050
void mpu6050_begin() {
  Wire.begin();
  Serial.print("MPU6050: Starting calibration; leave device flat and still... ");
  int error = mpu6050.begin();  // Uses the default 500 calibration samples
  Serial.println(mpu6050.error_str(error));
}

// Function to read yaw from MPU6050
float mpu6050_yaw() {
  MPU6050_t data = mpu6050.get();
  while (data.dir.error != 0) {
    Serial.println(mpu6050.error_str(data.dir.error));
    // Reset I2C and reread the data
    TWCR = 0;
    Wire.begin();
    data = mpu6050.get();
  }
  return data.dir.yaw;
}

// PID initialization
void pid_begin() {
  i_input = 0;
  d_last = 0;
  Serial.println("PID control initialized");
}

// PID control function
int pid(float error) {
  float p_input = error;
  i_input = constrain(i_input + error, -50, 50);  // Constrain integral sum
  float d_input = error - d_last;
  d_last = error;
  
  return p_input * PID_K_p + i_input * PID_K_i + d_input * PID_K_d;
}

// Go forward with PID control
void go_forward(float target_yaw) {
  // Read current yaw from MPU6050
  float current_yaw = mpu6050_yaw();
  
  // Calculate error between target and current yaw
  float error = target_yaw - current_yaw;
  
  // Calculate correction using PID control
  int correction = pid(error);
  
  // Adjust servo angle based on correction
  int servo_angle = SERVO_NEUTRAL - correction;
  servo_angle = constrain(servo_angle, 0, 180);
  thrustServo.write(servo_angle);
  
#if DEBUG
  Serial.print("Current yaw: ");
  Serial.print(current_yaw, 2);
  Serial.print(", Target yaw: ");
  Serial.print(target_yaw, 2);
  Serial.print(", Error: ");
  Serial.print(error, 2);
  Serial.print(", Correction: ");
  Serial.print(correction);
  Serial.print(", Servo angle: ");
  Serial.println(servo_angle);
#endif

  delay(20);  // Small delay for stability
}

// Apply PID control for a specific time
void just_PID(int iterations) {
  for (int i = 0; i < iterations; i++) {
    go_forward(targetYaw);
  }
}

// Turn functions
void setNewTargetYaw(float target) {
  targetYaw = target;
}

void fullTurn(bool turnRight) {
  if (turnRight) {
    setNewTargetYaw(-180);  // Right turn target
  } else {
    setNewTargetYaw(0);     // Left turn target
  }
#if DEBUG
  Serial.print("Making a ");
  Serial.print(turnRight ? "right" : "left");
  Serial.print(" turn to yaw: ");
  Serial.println(targetYaw);
#endif
}

// Look in a direction to measure distance
float lookInDirection(bool lookRight) {
  // Stop fan temporarily for more accurate readings
  digitalWrite(THRUST_FAN_PIN, LOW);
  
  // Turn servo to look in that direction
  thrustServo.write(lookRight ? RIGHT_ANGLE : LEFT_ANGLE);
  delay(100);  // Give servo time to turn
  
  // Measure distance
  float distance = frontSensor.dist();
  
  // Reset servo and restart fan
  thrustServo.write(SERVO_NEUTRAL);
  digitalWrite(THRUST_FAN_PIN, HIGH);
  
  return distance;
}

// Make a decision based on distance readings
bool makeDecision() {
  // Look left
  float leftDist = lookInDirection(left);
  delay(50);
  
  // Look right
  float rightDist = lookInDirection(right);
  delay(50);
  
#if DEBUG
  Serial.print("Left distance: ");
  Serial.print(leftDist);
  Serial.print(" | Right distance: ");
  Serial.println(rightDist);
#endif
  
  // Return true for right turn, false for left turn
  return rightDist > leftDist;
}

// Make a turn decision with logic from the winning implementation
void makeSmartDecision() {
  static bool initialDecisionMade = false;
  static bool initialTurnRight;
  
  if (!initialDecisionMade) {
    initialTurnRight = makeDecision();
    initialDecisionMade = true;
    turnCount = initialTurnRight ? 0 : 1;  // Set initial count based on first turn
#if DEBUG
    Serial.print("Initial decision: turn ");
    Serial.println(initialTurnRight ? "right" : "left");
#endif
  }
  
  turnCount++;
  
  if (turnCount % 2) {
    // Odd count - turn right
    fullTurn(right);
  } else {
    // Even count - turn left
    fullTurn(left);
  }
  
  // Check if we've reached the exit turn count
  if (turnCount == exitTurnCount || turnCount == exitTurnCount + 1) {
    WALL_THRESHOLD = EXIT_THRESHOLD;
#if DEBUG
    Serial.println("Now looking for exit with extended threshold");
#endif
  }
}

void setup() {
  // Initialize serial if debugging
#if DEBUG
  Serial.begin(115200);
  Serial.println("Improved Navigation System Initializing...");
#endif

  // Initialize MPU6050
  mpu6050_begin();
  
  // Initialize PID control
  pid_begin();
  
  // Initialize servo
  thrustServo.attach(SERVO_PIN);
  thrustServo.write(SERVO_NEUTRAL);
  
  // Initialize fan pins
  pinMode(THRUST_FAN_PIN, OUTPUT);
  pinMode(LIFT_FAN_PIN, OUTPUT);
  
  // Turn on fans
  digitalWrite(LIFT_FAN_PIN, HIGH);    // Start lift fan
  delay(500);                          // Let the cushion of air build up
  digitalWrite(THRUST_FAN_PIN, HIGH);  // Start thrust fan
  
  // Set initial target
  targetYaw = 0.0;
  
  delay(1000);  // Final stabilization
#if DEBUG
  Serial.println("Initialization complete. Starting navigation.");
#endif
}

// Main threshold for wall detection
int WALL_THRESHOLD = 40;

void loop() {
  // Check for walls/obstacles
  float distance = frontSensor.dist();
  
#if DEBUG
  Serial.print("Front distance: ");
  Serial.println(distance);
#endif
  
  if (distance >= WALL_THRESHOLD || distance == 0.0) {
    // No wall detected or sensor out of range, continue forward with PID control
    go_forward(targetYaw);
  } else {
    // Wall detected, make a turn decision
#if DEBUG
    Serial.println("Wall detected! Making turn decision...");
#endif
    makeSmartDecision();
    
    // Move forward a bit without checking distance to clear the turn
    just_PID(50);  // Apply PID for several iterations
  }
  
  delay(20);  // Small loop delay for stability
}
