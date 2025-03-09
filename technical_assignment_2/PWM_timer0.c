#include <avr/io.h>
#include <util/delay.h>

#define SERVO_MIN 1000  // 1ms pulse width (0° position)
#define SERVO_MAX 2000  // 2ms pulse width (180° position)

double servo_angle = 90; // Start at center position

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

    // Start at 90° (middle position)
    setServoAngle(servo_angle);
}

int main(void) {
    init_servo();

    while (1) {
        // Move servo from 0° to 180° in small steps
        for (int i = 0; i <= 180; i++) {
            setServoAngle(i);
            _delay_ms(10); // Wait for the servo to reach the position
        }
        
        // Move servo back from 180° to 0° in small steps
        for (int i = 180; i >= 0; i--) {
            setServoAngle(i);
            _delay_ms(10); // Wait for the servo to reach the position
        }
    }
}
