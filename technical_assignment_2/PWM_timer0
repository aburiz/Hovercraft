#define F_CPU 16000000UL 

#include <avr/io.h>
#include <avr/interrupt.h>

double dutyCycle = 0;

void pwm_init(void) {

    // set fast PWM mode
    TCCR0A = (1 << COM0A1) | (1 << WGM00) | (1 << WGM01);
    TIMSK0 = (1 << TOIE0);

    // calculate TON from duty cycle
    OCR0A = (dutyCycle/100) * 255;

    // enable external interrupts
    sei();

    // set prescaler to 1, start timer
    TCCR0B = (1 << CS00);
}

ISR(TIMER0_OVF_vect) {

    OCR0A = (dutyCycle/100) * 255;
}

int main(void) {

    while(1) {

    }
}
