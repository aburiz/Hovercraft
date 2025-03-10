#include <Wire.h>
#include <MPU6050.h>
#include "UART.c"

/********************************************************************************
*********************************** SERVO MOTOR *********************************
*********************************************************************************/

#define SERVO_MIN 1000  // 1ms pulse width (0° position)
#define SERVO_MAX 2000  // 2ms pulse width (180° position)
double servo_angle = 90; // Start at center position

volatile uint16_t counter = 0;

void setServoAngle(double angle) {

    if (angle < 9 || angle > 171) { // If outside ±81° range
        OCR1A = (angle < 9) ? SERVO_MIN : SERVO_MAX; // Set to min/max
        PORTB |= (1 << PORTB5);  // Turn LED ON (PB5)
    } else {
        OCR1A = SERVO_MIN + ((angle / 180.0) * (SERVO_MAX - SERVO_MIN)); // Calculate PWM
        PORTB &= ~(1 << PORTB5); // Turn LED OFF (PB5)
    }
}

void init_servo(void) {
    DDRB |= (1 << PB1) | (1 << PB5); // Set PB1 (OC1A) as output

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
************************************* MPU6050 ***********************************
*********************************************************************************/

MPU6050 mpu;
int16_t ax, ay, az, gx, gy, gz;

void init_mpu(void) {
    
    Wire.begin();
    print_message("Initializing MPU6050...."); print_message("\n");
    mpu.initialize();

    if (mpu.testConnection()) {
        print_message("MPU6050 connection successful."); print_message("\n");
    } else {
        print_message("MPU6050 connection failed. Check wiring and ensure AD0 is connected to ground."); print_message("\n");
        while (1); // Halt execution if sensor is not found.
    }
}

int main(void) {

    init_servo();
    init_uart();
    init_mpu();

    while (1) {
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        
        if (counter >= 50) {
            counter = 0;

            print_message("aX = ");     print_number(ax);
            print_message(" | aY = ");  print_number(ay);
            print_message(" | aZ = ");  print_number(az);
            print_message(" || gX = "); print_number(gx);
            print_message(" | gY = ");  print_number(gy);
            print_message(" | gZ = ");  print_number(gz);
            print_message("\n");

        }
    }
}
