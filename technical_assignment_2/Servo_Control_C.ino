#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

void initialize_servo_timer(){ // uses Timer 1 for 16 bits 
  DDRB |= (1 << PB1);  // set PB1 OC1A as servo output

  // set timer to fast PWM to clear OC1A on match to use top value (ICR1 which will set the PWM frequency) 
  TCCR1A |= (1<<COM1A1) | (1<<WGM11);  // set PB1 PMW to the timer
  TCCR1B |= (1<<WGM13) | (1<<WGM12) | (1<<CS11); // prescaler set to 8, clock frequency = 16MHz/8 = 2MHz, so PWM frequecny = 50Hz and fits in 16 bit range
  ICR1 = 19999; // set PWM to 50Hz = (16MHz / (8 * 50)) - 1
  OCR1A = 1500; // initialize angle to 90 degrees = 1000 + (angle*1000)/180
}

void write_servo(uint8_t angle){ // converts angle to pulse width, sets it with OCR1A Timer 1 PWM
  uint16_t pulse = 1000 + ((angle*1000)/180);
  OCR1A = pulse;
}

int main(){

  initialize_servo_timer();

  while(1){
    write_servo(0);
    _delay_ms(1000);
    write_servo(90);
    _delay_ms(1000);
    write_servo(180);
    _delay_ms(1000);
    write_servo(90);
    _delay_ms(1000);
  }
}

