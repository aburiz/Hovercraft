#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <math.h>

#include <Wire.h>
#include <MPU6050.h>
#include <util/delay.h>

/********************************************************************************
*********************************** UART ****************************************
*********************************************************************************/

// for serial communication (UART)
#define F_CPU 16000000UL 
#define baud 9600
#define ubrr ((F_CPU / (16UL * baud)) - 1)

void init_uart(){
    // setting baud rate with UBRR0
    UBRR0H = (uint8_t)(ubrr >> 8);
    UBRR0L = (uint8_t)ubrr;
    // setting UART to transmission
    UCSR0B = (1 << TXEN0);
    // setting UART to 8-bit character size for transmission
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void transmit_uart(char data){
    while(!(UCSR0A & (1<<UDRE0))); // wait for transmission to be empty (flag to be 1)
    UDR0 = data; // transmit data
}

void print_message(const char *message){ //using UART
    while(*message){ 
        transmit_uart(*message++);
    }
 }

void print_number(int16_t number){
    char string_num[8]; // max number in 16 bits is 6 bits long + 2 bits for space
    uint8_t i = 0;

    if(number == 0){
      transmit_uart('0');
      return;
    }

    //only positive value readings from the IR sensor
    // convert number into string, store in string_num
    while(number > 0){
      string_num[i++] = (number%10) + '0'; // store last digit it in ASCII
      number /= 10; 
    }

    // transmit (print) each character in the string number array
    while(i>0){
      transmit_uart(string_num[--i]);
    }
}

/********************************************************************************
*********************************** MPU6050 *************************************
*********************************************************************************/

#define YAW_MAX 81   // Maximum allowed yaw degrees
#define YAW_MIN -81  // Minimum allowed yaw degrees
#define ACC_THRESHOLD_LOW  0.12   // Below this (g): LED off
#define ACC_THRESHOLD_HIGH 1.12    // Above this (g): LED fully on

// --- Sensor conversion factors ---
#define ACCEL_SENSITIVITY 16384.0  // For ±2g mode (LSB/g)
#define GYRO_SENSITIVITY  131.0     // For ±250°/sec mode (LSB/(degree/sec))

#define RADS 57.2958

// --- Filter and integration constants ---
const float compFilterAlpha = 0.98;  // Complementary filter weight for gyro
const float VELOCITY_THRESHOLD = 0.01;     // Deadband for velocity (m/s)

MPU6050 mpu;

// Orientation in radians
float fusedRoll = 0.0, fusedPitch = 0.0, fusedYaw = 0.0;
bool firstOrientation = true;
unsigned long lastTimeOrientation = 0;

// Displacement integration variables (world X axis)
float distanceX = 0.0;
float velocityX = 0.0;
unsigned long lastDistanceTime = 0;

// Accelerometer bias for X axis (in g's)
float accelBiasX = 0.0;

// Output interval in ms
const unsigned long OUTPUT_INTERVAL = 1000;
unsigned long lastOutputTime = 0;

float ax_corr, ay_corr, az_corr, accRoll, accPitch;
float gyroX, gyroY, gyroZ;
float yaw_deg;
float ax_mps2, ay_mps2, az_mps2;
float g_sensor_x, g_sensor_y, g_sensor_z;
float ax_lin, ay_lin;
float a_world_x;

int16_t ax, ay, az, gx, gy, gz;

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

// Correct accelerometer readings (in g's)
void corr_accel_readings() {

    ax_corr = (ax / ACCEL_SENSITIVITY) - accelBiasX;
    ay_corr = (ay / ACCEL_SENSITIVITY);
    az_corr = (az / ACCEL_SENSITIVITY);
}

// Gyro readings in deg/s, convert to rad/s
void corr_gyro_readings() {
    gyroX = (gx / GYRO_SENSITIVITY) * RADS;
    gyroY = (gy / GYRO_SENSITIVITY) * RADS;
    gyroZ = (gz / GYRO_SENSITIVITY) * RADS;
}

// Compute accelerometer-based angles (in radians)
void compute_angles() {
    accRoll = atan2(ay_corr, az_corr);
    accPitch = atan2(-ax_corr, sqrt(ay_corr * ay_corr + az_corr * az_corr));
}

// Sensor Fusion for Displacement 
void sensor_fusion() {

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
    unsigned long currentDistanceTime = ms();
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
}

/********************************************************************************
*********************************** SERVO MOTOR *********************************
*********************************************************************************/

#define SERVO_MIN 1000  // 1ms pulse width (0° position)
#define SERVO_MAX 2000  // 2ms pulse width (180° position)
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
        OCR1A = SERVO_MIN + ((angle / 180.0) * (SERVO_MAX - SERVO_MIN)); // Calculate PWM
        PORTB &= ~(1 << PORTB5); // Turn LED OFF (PB5)
    }
}

void init_servo(void) {
    DDRB |= (1 << PB1) | (1 << PB5); // Set PB1 (OC1A) as output and L

    // Configure Timer1 for Fast PWM (Mode 14)
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

void init_ms_timer(){ 

    TCCR0A |= (1 << COM0A1) | (1 << WGM01);
    OCR0A = 249;
    
    // Enable Timer0 Compare Match A Interrupt
    TIMSK0 |= (1 << OCIE0A);

    sei();

    TCCR0B |= (1 << CS01) | (1 << CS00);
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

int main() {
    
    init_uart();
    init_servo();
    Wire.begin();
    
    mpu.initialize();
    if (!mpu.testConnection()) {
        print_message("MPU6050 connection failed!");   
        while (1);
    }

    // Set sensor full-scale ranges
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);

    lastTimeOrientation = ms();
    lastDistanceTime = ms();
    lastOutputTime = ms();

    calibrate_accelerometer();

    while (1) {
        // Orientation Update 
        unsigned long currentTime = ms();
        float dt = (currentTime - lastTimeOrientation) / 1000.0;
        if (dt <= 0) dt = 0.01;
        lastTimeOrientation = currentTime;

        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        corr_accel_readings();
        corr_gyro_readings();
        compute_angles();

        // Complementary filter for roll and pitch, integrate gyro for yaw
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
        yaw_deg = fusedYaw * RADS;   
        setServoAngle(yaw_deg);

        sensor_fusion();

        // Output Data Once Per Second 
        if (counter >= 50) {
            counter = 0;
            print_message("Roll: "); Serial.print(fusedRoll * RADS, 2);
            print_message("  Pitch: "); Serial.print(fusedPitch * RADS, 2);
            print_message("  Yaw: "); Serial.print(yaw_deg, 2);
            print_message("  AccX: "); Serial.print(ax_corr, 2);
            print_message("  AccY: "); Serial.print(ay_corr, 2);
            print_message("  AccZ: "); Serial.print(az_corr, 2);
            print_message("  Distance X: "); Serial.print(distanceX, 4);
            print_message(" m");
        }
    }
}
