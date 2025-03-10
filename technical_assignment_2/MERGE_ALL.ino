#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <math.h>

#include <Wire.h>
#include <MPU6050.h>
#include <util/delay.h>

#define DEG_TO_RADS 3.141592653/180
#define RAD_TO_DEGS 180/3.141592653

/********************************************************************************
*********************************** UART ****************************************
*********************************************************************************/

// for serial communication (UART)
#define F_CPU 16000000UL 
#define BAUD 9600
#define UBRR_VALUE ((F_CPU / (16UL * BAUD)) - 1)

void initialize_uart() {
    // setting baud rate with UBRR0
    UBRR0H = (uint8_t)(UBRR_VALUE >> 8);
    UBRR0L = (uint8_t)UBRR_VALUE;
    // setting UART to transmission
    UCSR0B = (1 << TXEN0);
    // setting UART to 8-bit character size for transmission
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void transmit_uart(char data) {
    while (!(UCSR0A & (1 << UDRE0))); // wait for transmission to be empty (flag to be 1)
    UDR0 = data; //transmit data
}

void print_message(const char *message) { //using UART
    while (*message) { 
        transmit_uart(*message++);
    }
}

void print_number(double number) {
    if (number < 0) {
        transmit_uart('-');
        number = -number;
    }

    //  integer part
    uint16_t int_part = (uint16_t)number;
    char buffer[8]; 
    uint8_t i = 0;

    if (int_part == 0) {
        transmit_uart('0');
    } else {
        while (int_part > 0) {
            buffer[i++] = (int_part % 10) + '0';
            int_part /= 10;
        }
        while (i > 0) {
            transmit_uart(buffer[--i]);
        }
    }

    // decimal point
    transmit_uart('.');

    // get decimals
    double decimal = (number - (uint16_t)number) * 10000.0 + 0.5; 
    uint16_t decimal_part = (uint16_t)decimal;

    for (int j = 1000; j > 0; j /= 10) {
        transmit_uart((decimal_part / j) % 10 + '0');
    }
}

/********************************************************************************
*********************************** LED *************************************
*********************************************************************************/

// minimum and maximum range values for x-axis acceleration (in g) for LED D3
double ACCEL_MIN = 0.12; 
double ACCEL_MAX = 1.12; 

void initialize_led() {
    // set PB3 (Pin 11) as output
    DDRB |= (1 << PB3);
    // PWM timer 2 configurations
    TCCR2A = (1 << COM2A1) | (1 << WGM21) | (1 << WGM20);
    TCCR2B = (1 << CS21); // prescaler = 8

    OCR2A = 0; 
}

void write_led(uint8_t brightness) {
    OCR2A = brightness;
}

uint8_t led_brightness(double acc) {
    
    if(absolute(acc) < 0.12){
      // print_message("LED : ");
      // print_number(0);
      // print_message("\t");

      return 255;
    }
    else if(absolute(acc) > 1.12){
      // print_message("LED : ");
      // print_number(100);
      // print_message("\t");
      return 0;
    }

    // normalize acceleration to percentage
    double intensity = map(absolute(acc), ACCEL_MIN, ACCEL_MAX, 0, 100); 

    // print_message("LED : ");
    // print_number(intensity);
    // print_message("\t");

    return map(intensity, 0.0, 100.0, 255, 0);
}

/********************************************************************************
*********************************** MATHS *************************************
*********************************************************************************/

double absolute(double num){
   double abs =(num<0) ? num*(-1) : num;
   return abs;
}

double map(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/********************************************************************************
*********************************** MPU6050 *************************************
*********************************************************************************/

#define YAW_MAX 81   // Maximum allowed yaw 
#define YAW_MIN -81  // Minimum allowed yaw 

// Sensor conversion factors 
double ACCEL_SENSITIVITY = 0; 
double GYRO_SENSITIVITY  = 0;     

#define RADS 57.2958

#define accRange 2
#define gyroRange 250

double x = 0.0; // for distance calculation

// Filter and integration constants 
const float compFilterAlpha = 0.98;  // Complementary filter weight for gyro
const float VELOCITY_THRESHOLD = 0.01;     // Deadband for velocity m/s

MPU6050 mpu;

// Orientation in radians
float fusedRoll = 0.0, fusedPitch = 0.0, fusedYaw = 0.0;
bool firstOrientation = true;
unsigned long lastTimeOrientation = 0;

// Displacement integration variables (world X axis)
float distanceX = 0.0;
float velocityX = 0.0;
unsigned long lastDistanceTime = 0;

// Accelerometer bias for X axis in g's
float accelBiasX = 0.0;

// Output interval in ms
const unsigned long OUTPUT_INTERVAL = 1000;
unsigned long lastOutputTime = 0;

// Accelerometer Calibration (X-axis) 
void calibrate_accelerometer() {
    
    const int numSamples = 100;
    float sumX = 0.0;
    for (int i = 0; i < numSamples; i++) {
        int16_t ax, ay, az, gx, gy, gz;
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        float rawAccelX = ax / ACCEL_SENSITIVITY;
        sumX += rawAccelX;
        _delay_ms(5);
    }

    accelBiasX = sumX / numSamples;
    print_message("Calibrated accelBiasX: ");
    Serial.println(accelBiasX, 4);
}

/********************************************************************************
*********************************** SERVO MOTOR *********************************
*********************************************************************************/

#define SERVO_MIN 2000  // 1ms pulse width 0° position
#define SERVO_MAX 4000  // 2ms pulse width 180° position
double servo_angle = 90; // Start at center position

volatile uint16_t counter = 0;

void setServoAngle(double angle) {

    if(angle < 9) {
        OCR1A = SERVO_MIN;
        PORTB |= (1 << PORTB5);
    } else if (angle >171) {
        OCR1A = SERVO_MAX;
        PORTB |= (1 << PORTB5);
    } else {
        OCR1A = map(angle, 0, 180, SERVO_MIN, SERVO_MAX); // Calculate PWM
        PORTB &= ~(1 << PORTB5); // Turn LED OFF (PB5)
    }
}

void init_servo(void) {
    DDRB |= (1 << PB1) | (1 << PB5); // Set PB1 (OC1A) as output and L

    // Configure Timer1 for Fast PWM Mode 14
    TCCR1A = (1 << COM1A1) | (1 << WGM11);
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11); // Prescaler = 8

    // Set PWM frequency to 50 Hz
    ICR1 = 40000; 

    // Enable Timer1 Overflow Interrupt
    TIMSK1 |= (1 << TOIE1);
    sei();

    // Start at 90° (middle position)
    setServoAngle(servo_angle);
}

ISR(TIMER1_OVF_vect) {
    
    counter++;
}

/********************************************************************************
************************************** MILLIS() *********************************
*********************************************************************************/
volatile uint32_t milliseconds = 0;

void init_ms_timer() {

    TCCR0A = (1 << WGM01);  // Enable CTC mode
    OCR0A = (F_CPU / (64 * 1000)) - 1;            // Set compare match for 1ms

    TIMSK0 |= (1 << OCIE0A); // Enable Timer0 Compare Match A Interrupt
    TCCR0B |= (1 << CS01) | (1 << CS00); // Start Timer0 with Prescaler 64

    sei(); // Enable global interrupts AFTER starting timer
}

ISR(TIMER0_COMPA_vect) {

    milliseconds++;
}

uint32_t ms() { 
    uint32_t ms;
    cli(); // pause interrupts to get reading;
    ms = milliseconds;
    sei(); //restart interrupts
    return ms;
}

/********************************************************************************
************************************** MAIN *************************************
*********************************************************************************/

void setup(){
  initialize_uart();
  initialize_led();
  init_ms_timer();
  init_servo();
  Wire.begin();
  
  mpu.initialize();
  if (!mpu.testConnection()) {
      print_message("MPU6050 connection failed!");   
      while (1);
  }

switch(gyroRange){
    case(250):
      mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
      GYRO_SENSITIVITY = 131;
    
    case(500):
      mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);
      ACCEL_SENSITIVITY = 65.5;
    
    case(1000):
      mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_1000);
      ACCEL_SENSITIVITY = 32.8;
    
    case(2000):
       mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
       ACCEL_SENSITIVITY = 16.4;
    
  }

  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  switch(accRange){
    case(2):
      mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2); // +-2g (16384 LSB/g)
      ACCEL_SENSITIVITY = 16384;
    case(4):
      mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);  // +-4g (8192 LSB/g)
      ACCEL_SENSITIVITY = 8192;
    case(8):
      mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);  // +-8g (4096 LSB/g)
      ACCEL_SENSITIVITY = 4096;
    case(16):
       mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_16); // +-2g (2048 LSB/g)
       ACCEL_SENSITIVITY = 2043;
    
  }

  lastTimeOrientation = ms();
  lastDistanceTime = ms();
  lastOutputTime = ms();

  calibrate_accelerometer();


}

void loop(){
  // Orientation Update 
  unsigned long currentTime = ms();
  float dt = (currentTime - lastTimeOrientation) / 1000.0;
  if (dt <= 0) dt = 0.01;
  lastTimeOrientation = currentTime;

  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Correct accelerometer readings in g's
  float ax_corr = (ax / ACCEL_SENSITIVITY) - accelBiasX;
  float ay_corr = (ay / ACCEL_SENSITIVITY);
  float az_corr = (az / ACCEL_SENSITIVITY);

  // Gyro readings in deg/s, convert to rad/s
  float gyroX = (gx / GYRO_SENSITIVITY) * DEG_TO_RADS;
  float gyroY = (gy / GYRO_SENSITIVITY) * DEG_TO_RADS;
  float gyroZ = (gz / GYRO_SENSITIVITY) * DEG_TO_RADS;

  // Compute accelerometer based angles (in radians)
  float accRoll = atan2(ay_corr, az_corr);
  float accPitch = atan2(-ax_corr, sqrt(ay_corr * ay_corr + az_corr * az_corr));


  // Complementary filter for roll and pitch; integrate gyro for yaw
  if (firstOrientation) {
      fusedRoll = accRoll;
      fusedPitch = accPitch;
      fusedYaw = 0.0;
      firstOrientation = 0;
  } else {
      fusedRoll = compFilterAlpha * (fusedRoll + gyroX * dt) + (1 - compFilterAlpha) * accRoll;
      fusedPitch = compFilterAlpha * (fusedPitch + gyroY * dt) + (1 - compFilterAlpha) * accPitch;
      fusedYaw = fusedYaw + gyroZ * dt;  // Yaw from gyro integration only
  }

// Servo Control Based on Yaw 
  float yaw_deg = fusedYaw * RAD_TO_DEG;
  setServoAngle(map(yaw_deg, -81, 81, 0, 180));

  // Sensor Fusion for Displacement
  // Convert corrected accelerometer readings to m/s^2
  float ax_mps2 = ax_corr * 9.81;
  float ay_mps2 = ay_corr * 9.81;
  float az_mps2 = az_corr * 9.81;

  // Compute expected gravity vector in sensor frame from fused roll and pitch
  float g_sensor_x = -sin(fusedPitch) * 9.81;
  float g_sensor_y = sin(fusedRoll) * cos(fusedPitch) * 9.81;
  float g_sensor_z = cos(fusedRoll) * cos(fusedPitch) * 9.81;

  // Subtract gravity to obtain linear acceleration 
  float ax_lin = ax_mps2 - g_sensor_x;
  float ay_lin = ay_mps2 - g_sensor_y;
  // We'll use only the horizontal components for displacement

  // Rotate horizontal (x-y) linear acceleration by fused yaw to align with world X–axis
  float a_world_x = ax_lin * cos(fusedYaw) - ay_lin * sin(fusedYaw);

  // Distance Integration (Trapezoidal with Zero Velocity Update) 
  unsigned long currentDistanceTime = ms();
  float dtDistance = (currentDistanceTime - lastDistanceTime) / 1000.0;
  lastDistanceTime = currentDistanceTime;

  // Integrate a_world_x to update velocity using trapezoidal rule
  static float prevAccX = 0.0;
  float newVelocityX = velocityX + 0.5 * (prevAccX + a_world_x) * dtDistance;
  if (fabs(newVelocityX) < VELOCITY_THRESHOLD)
      newVelocityX = 0.0;

  // Zero velocity update: if acceleration remains near zero for a period, reset velocity
  const float ACCEL_DRIFT_THRESHOLD = 0.05; // m/s^2 threshold for stationary detection
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


  // LED intensity based on x acceleration
  write_led(led_brightness(ax_corr)); 

  static unsigned long lastSampleTime = ms();
  unsigned long timeNow = ms();

  double dist = (timeNow - lastSampleTime) / 1000.0; // in seconds
  lastSampleTime = timeNow;

  if (fabs(ax_corr) < 0.02) ax_corr = 0;
  double p = (0.5 * ax_corr * 9.80665 * (dist * dist)) * 100;  // now in cm
  x = x + p;

  // Output Data Once Per Second
  if (counter >= 50) {
      counter = 0;
      print_message("Roll: "); print_number(fusedRoll * RAD_TO_DEG);
      print_message("\tPitch: "); print_number(fusedPitch * RAD_TO_DEG);
      print_message("\t AccX: "); print_number(ax_corr);
      print_message("\t Yaw: "); print_number(yaw_deg);
      print_message("\tAccY: "); print_number(ay_corr);
      print_message("\tAccZ: "); print_number(az_corr);
      print_message("\tDistance X: "); print_number(x);
      print_message(" cm\n");
  }
}
