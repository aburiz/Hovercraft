#include <Wire.h>
#include <MPU6050.h>
#include "UART.c"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <math.h>

/********************************************************************************
************************************* UART **************************************
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

  void print_number(uint16_t number){
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
************************************* MPU6050 ***********************************
*********************************************************************************/

MPU6050 mpu;
int16_t ax, ay, az, gx, gy, gz;
double yaw = 0;

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

    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    yaw = yaw + gz*(1/50);
    setServoAngle(yaw);
}

int main(void) {

    init_servo();
    init_uart();
    init_mpu();

    while (1) {
        
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
