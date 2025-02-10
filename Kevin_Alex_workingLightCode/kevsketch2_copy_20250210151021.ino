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

#define SENSOR_MIN 12.0
#define SENSOR_MAX 46.0

const long BLINK_INTERVAL = 500;  
int led_state = LOW; 
unsigned long prev_time_ms = 0; 

void blink(unsigned long time_ms) {
   if (time_ms - prev_time_ms >= BLINK_INTERVAL){
        led_state = (led_state == LOW) ? HIGH : LOW;
        digitalWrite(L, led_state);
        prev_time_ms = time_ms;
    }
}

double measure_IR(int adc) {
    double voltage = adc * VRef / 1023;
    return 29.988 * pow(voltage, -1.173);
}

double measure_US() {

    digitalWrite (trigPin, HIGH);
    delay(50);
    digitalWrite (trigPin, LOW);
    int duration=pulseIn(echoPin,HIGH);
    int distance=(duration/2)/29.1;

    return distance;
}

int LED_brightness(double distance) {
    double intensity = ((distance - SENSOR_MIN) / (SENSOR_MAX - SENSOR_MIN)) * 100;

    Serial.print("\tLED : ");
    Serial.println(intensity);

    return map(intensity, 0, 100, 0, 229.5); // maps from 0 - 255
}

void setup() {

    Serial.begin(BAUD_RATE);

    pinMode(L, OUTPUT);
    pinMode(LED, OUTPUT);
}

void loop() {

    unsigned long time_ms = millis();

    IR_dist = measure_IR(analogRead(IR_pin));
    Serial.print("IR Distance : ");
    Serial.print(IR_dist);

    // US_dist = measure_US();
    // Serial.print("\tUSS Distance : ");
    // Serial.print(US_dist);

    if(IR_dist < 12) {
      analogWrite(LED, LED_brightness(SENSOR_MIN));
      blink(time_ms);
    } else if (IR_dist > 46) {
      analogWrite(LED, LED_brightness(SENSOR_MAX));
      blink(time_ms);
    } else {
      analogWrite(LED, LED_brightness(IR_dist));
    }
}

 
