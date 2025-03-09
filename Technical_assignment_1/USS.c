
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>

typedef unsigned int bool;
#define true 1
#define false 0

// for serial communication (UART)
#define F_CPU 16000000UL 
#define baud 9600
#define ubrr ((F_CPU / (16UL * baud)) - 1)

// minimum and maximum range values
#define SENSOR_MIN 12.0
#define SENSOR_MAX 46.0

// for US sensor
#define trig_pin PB5  // pin 13 (same as L)
#define echo_pin PD3  // pin 3

// variables for implementing blinking L
uint32_t BLINK_INTERVAL = 100;  
bool led_state = false; 
uint32_t prev_time_ms = 0; 
// variable for implementing millis() using timer and interrupts
volatile uint32_t milliseconds = 0;

void initialize_uart(){
  // setting baud rate with UBRR0
  UBRR0H = (uint8_t)(ubrr >> 8);
  UBRR0L = (uint8_t)ubrr;
  // setting UART to transmission
  UCSR0B = (1 << TXEN0);
  // setting UART to 8-bit character size for transmission
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void initialize_adc_ir(){ // uses A0 for IR sensor
  // we want to set Vref to Aref pin for personalized reference voltage for the IR sensor (3V)
  // so REFS[1:0] in ADMUX should be 00
  ADMUX &= ~((1<<REFS1) | (1<<REFS0)); // clear REFS[1:0] to use VREF 
  ADMUX = (ADMUX & 0xF0); // use A0 (MUX[3:0} = 0000])
  ADCSRA = (1<<ADEN) | (1<< ADPS2)| (1<< ADPS1)| (1<< ADPS0); // enable ADC set ADC speed to 128 (accurate)
}

void initialize_led(){
   // PB3 (pin 11) is the LED
  DDRB |= (1 << PB3); // set PB3 as output
  // PMW timer configurations
  TCCR2A = (1 << COM2A1) | (1 << WGM21) | (1 << WGM20); 
  TCCR2B = (1 << CS21); 

  OCR2A = 0; 
}

void initialize_us() {
    DDRB |= (1 << trig_pin); // output
    DDRD &= ~(1 << echo_pin); // input
}

void initialize_ms_timer(){ // this timer will tick once every 4us (prescaler = 64) and send an interrupt every 1ms
  TCCR1B |= (1<<WGM12) | (1<<CS11) | (1<<CS10); // WGM12 enables clear time on compare mode for timer, CS1[1:0] sets prescaler to 64 to slow down clock
  OCR1A = 249; // sets the precise value when interrupt will be sent (every 1ms, calculated with the CPU frequency and prescaler)
  TIMSK1 |= (1 << OCIE1A); // allows output compare interrupt for timer 1
  sei(); // allows global interrupts
}

ISR(TIMER1_COMPA_vect){ // interrupt service routine for when timer1 sends an interrupt (count milliseconds)
  milliseconds++;
}

uint32_t millis() { // making our own millis() function to count milliseconds for blinking LED without delaying code
  uint32_t ms;
  cli(); // pause interrupts to get reading;
  ms = milliseconds;
  sei(); //restart interrupts
  return ms;
}

void write_led(uint8_t brightness){
  OCR2A = brightness;
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

void trigger_us() {
    PORTB |= (1 << trig_pin);  // set pin high
    _delay_us(10);              
    PORTB &= ~(1 << trig_pin); // set pin low
}

uint16_t pulse_length_us(){
  trigger_us(); // send trigger
    
    // wait for echo pin to go high
    while (!(PIND & (1 << echo_pin))){};

    // measure pulse in microseconds
    uint16_t time = 0;
    while (PIND & (1 << echo_pin)) { // read echo pin while high
        _delay_us(1); 
        time++;        
    }

    return time;
}

uint16_t distance_us(uint16_t duration) {
    uint16_t distance = ((duration / 2.0) / 29.1); // convert to distance
    return distance;
}

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

uint8_t led_brightness(uint16_t distance) {
    double intensity = ((distance - SENSOR_MIN) / (SENSOR_MAX - SENSOR_MIN)) * 100; // make intensity a percentage to debug

    print_message("\tLED : ");
    print_number(intensity);
    print_message("\n\r");

    return map(intensity, 0, 100, 0, 229.5); // maps from 0 - 229.5 (10% intensity min)
}

void blink(uint32_t time_ms) { // PB5 (pin 13) controls L 
   if (time_ms - prev_time_ms >= BLINK_INTERVAL){
        led_state = !led_state;
        if(led_state) PORTB |= (1<<PB5); // write high
        else PORTB &= ~(1<<PB5); // write low
        prev_time_ms = time_ms;
    }
}

int main(){
  
  // initialize US
  initialize_us();
  // initialize uart for serial communication (serialprint)
  initialize_uart();
  // initialize led to write analogue values to it using PMW
  initialize_led();
  // initialize timer to count milliseconds (for blink)
  initialize_ms_timer();
  // initialize L led to write digital values
  DDRB |= (1 << PB5);

  while(1){
    uint32_t time_ms = millis();

    uint16_t duration_US = pulse_length_us();
    uint16_t dist_US = distance_us(duration_US);
    print_message("US pulse length: ");
    print_number(duration_US);
    print_message("\tdistance: ");
    print_number(dist_US);
    

    if(dist_US < 12) {
      write_led(led_brightness(SENSOR_MIN));
      blink(time_ms);
    } else if (dist_US > 46) {
      write_led(led_brightness(SENSOR_MAX));
      blink(time_ms);
    } else {
      write_led(led_brightness(dist_US));
    }
  }

}
