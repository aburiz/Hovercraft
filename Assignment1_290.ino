#define BAUD_RATE 9600
#define VRef 4.68

// Light pins
#define L 13
#define LED 11

#define IR_pin A0

// ultrasonic sensor (USS)
#define trigPin 13  // arduino nano pins
#define echoPin 3  //  (from PB0) 

double IR_dist = 0;
double US_dist = 0;

int LED_bright = 0;
const long BLINK_INTERVAL = 200;  
int led_state = LOW; 
unsigned long prev_time_ms = 0; 


void setup() {
  Serial.begin(BAUD_RATE);

  pinMode(L, OUTPUT);
  pinMode(LED, OUTPUT);
}

void loop() {

  unsigned long time_ms = millis();

  IR_dist = cal_IR_distance(analogRead(IR_pin));
  Serial.print("IR Distance : ");
  Serial.print(IR_dist);

  US_dist = cal_US_distance();
  Serial.print("\tUSS Distance : ");
  Serial.print(US_dist);

  if(IR_dist >= 12 && IR_dist <= 46) { 
    LED_bright = LED_brightness(IR_dist);
    analogWrite(LED, LED_bright);
    digitalWrite(L, LOW);
  } else if(IR_dist >46){
    analogWrite(LED, 229.5); // 90% of 255
    blink(time_ms); 
  } else {
    digitalWrite(LED, 0); // 100% brightness
    blink(time_ms); 
  }

  Serial.print("\tLED : ");
  Serial.println(LED_bright);
  //delay(200);
}

void blink(unsigned long time_ms) {
   if (time_ms - prev_time_ms >= BLINK_INTERVAL){
      led_state = (led_state == LOW) ? HIGH : LOW;
      digitalWrite(L, led_state);
      prev_time_ms = time_ms;
    }
}

double cal_IR_distance(int adc) {
  double voltage = adc * VRef / 1023;
  return 29.988 * pow(voltage, -1.173);
}

double cal_US_distance() {

  digitalWrite (trigPin, HIGH);
  delay(50);
  digitalWrite (trigPin, LOW);
  int duration=pulseIn(echoPin,HIGH);
  int distance=(duration/2)/29.1;

  return distance;
}

int LED_brightness(double distance) {
  return map(distance, 12, 46, 0, 229.5); // maps from 100% - 90% intensity
}




// // - An IR sensor on ADC0 (PC0)
// #define F_CPU 16000000UL

// #include <avr/io.h>
// #include <avr/interrupt.h>
// #include <util/delay.h>

// // IR functions

// // Initializes the ADC to use AVcc as reference and selects channel 0 (PC0).
// void adc_init(void)
// {
//     // ADMUX: REFS0 = 1 for AVcc reference; MUX[3:0] = 0 for ADC0.
//     ADMUX = (1 << REFS0);
//     // ADCSRA: Enable ADC (ADEN) and set prescaler to 128 for 16MHz clock.
//     ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
// }

// // Performs a single ADC conversion on ADC0 and returns the 10-bit result.
// uint16_t adc_read(void)
// {
//     ADCSRA |= (1 << ADSC);          // start conversion
//     while (ADCSRA & (1 << ADSC));   // wait until conversion is complete
//     return ADC;
// }

// int main(void){

//   // initialization IR sensor
//   adc_init();  // Initialize ADC for channel 0 (PC0).


//   // equivalent to loop() in arduino
//   while(1){
//     // IR sensor reading
//     ir_value = adc_read();

//       if(distance > 10)
//         PORTD |= (1 << PD7);   // Turn LED ON.
//       else
//         PORTD &= ~(1 << PD7);  // Turn LED OFF.
//       _delay_ms(100);
  
  
  
//   }
// }



