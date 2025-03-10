// #include <Wire.h>
// #include <MPU6050.h>
#include <util/delay.h>
// #include "UART.c"

/**
** SERVO MOTOR **
**/


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

#define SERVO_MIN 2000  // 1ms pulse width (0° position)
#define SERVO_MAX 4000  // 2ms pulse width (180° position)
double servo_angle = 90; // Start at center position

volatile uint16_t counter = 0;

void setServoAngle(double angle) {
  OCR1A = map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
  // OCR1A = SERVO_MIN + ((angle / 180.0)*(SERVO_MAX - SERVO_MIN)); // Calculate PWM
}

void init_servo(void) {
    DDRB |= (1 << PB1); // Set PB1 (OC1A) as output

    // Configure Timer1 for Fast PWM (Mode 14)
    TCCR1A = (1 << COM1A1) | (1 << WGM11);
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11); // Prescaler = 8

    // Set PWM frequency to 50 Hz
    ICR1 = 40000;

    // Start at 90° (middle position)
    setServoAngle(servo_angle);
}

int main(void) {

    initialize_uart();
    init_servo();

    while (1) {

        // for (int i=0; i < 180; i++) {
        //     setServoAngle(i);
        // }
        // for (int i=180; i > 0; i--) {
        //     setServoAngle(i);
        // }

         setServoAngle(180);
        // _delay_ms(1000);
        // setServoAngle(180);
        // _delay_ms(1000);
    }
}
