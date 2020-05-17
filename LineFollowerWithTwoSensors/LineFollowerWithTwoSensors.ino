#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>

#define ROBOT_SPEED_RIGHT 70
#define ROBOT_SPEED_LEFT 50
#define TURN_RATE 25

// This code is designed for use with two sensors.
// The left sensor should be connected to PD7 (sensor 6).
// The right sensor should be connected to PF6 (sensor 3).

int main()
{
  initADC();
  initPWM();

  // Turn on IR LEDs.
  DDRB |= (1<<3);
  PORTB |= (1<<3);

  // Enable LED2 and LED3
  DDRB |= (1<<1)|(1<<2);

  while (1)
  {
    // Clear MUX bits
    ADMUX &= ~(0b00011111);
    ADCSRB &= ~(1<<5);
    // Set MUX bits to use ADC6 (sensor 3)
    ADMUX |= 0b00000110;
    triggerADC();
    uint8_t reading_right = ADCH;

    // Clear MUX bits
    ADMUX &= ~(0b00011111);
    ADCSRB &= ~(1<<5);
    // Set MUX bits to use ADC10 (sensor 6)
    ADMUX |= 0b00000010;
    ADCSRB |= (1<<5);
    triggerADC();
    uint8_t reading_left = ADCH;

    if (reading_left > reading_right){
      // Turn to the left
      PORTB |= (1<<1);
      PORTB &= ~(1<<2);
      OCR0A = ROBOT_SPEED_LEFT;
      OCR0B = ROBOT_SPEED_RIGHT + TURN_RATE;
    }
    else{
      // Turn to the right
      PORTB |= (1<<2);
      PORTB &= ~(1<<1);
      OCR0A = ROBOT_SPEED_LEFT + TURN_RATE;
      OCR0B = ROBOT_SPEED_RIGHT;
    }

    _delay_ms(50);
  }
}

// Display the four most significant bits of a number on LEDs 0-3.
// Use for debugging purposes.
void displayNumber(int num){
  // Set Data Direction bits for LEDs 0-3
  DDRE |= (1<<6);
  DDRB |= (1<<2) | (1<<1) | 1;

  if (num & (1<<7)){
    // Turn on LED0
    PORTE |= (1<<6);
  }
  else{
    // Turn off LED0
    PORTE &= ~(1<<6);
  }

  if (num & (1<<6)){
    // Turn on LED0
    PORTB |= (1);
  }
  else{
    // Turn off LED0
    PORTB &= ~(1);
  }

  if (num & (1<<5)){
    // Turn on LED0
    PORTB |= (1<<1);
  }
  else{
    // Turn off LED0
    PORTB &= ~(1<<1);
  }

  if (num & (1<<4)){
    // Turn on LED0
    PORTB |= (1<<2);
  }
  else{
    // Turn off LED0
    PORTB &= ~(1<<2);
  }
}

// Init PWM on both motors.
// Note that this will also start the motors immediately.
void initPWM(){
  // Set up timer with prescaler = 256
  TCCR0B |= (1<<2);

  // Set fast PWM mode
  TCCR0A |= (1<<1)|1;

  // Clear OC0A on Compare Match, set OC0A at TOP
  // Clear OC0B on Compare Match, set OC0B at TOP
  TCCR0A |= (1<<7)|(1<<5);

  // PWM control on motor B.
  DDRB |= (1<<7);  // Set B7 to output

  // PWM control on motor A.
  DDRD |= 1;  // Set D0 to output

  // Use this to control duty cycle
  // Must be under 255
  OCR0A = 150;
  OCR0B = 150;

  // Set direction of right motor.
  PORTE |= (1<<6);
}

// Init ADC before reading an analog voltage.
void initADC(){
  // Configure ADC to be left justified, and use AVCC as a reference
  ADMUX |= (1<<5) | (1<<6);

  // Enable ADC and set prescaler to max value (128)
  ADCSRA |= (1<<7) | 0b00000111;
}

// Start ADC conversion and wait for it to complete.
void triggerADC(){
  // Start ADC conversion
  ADCSRA |= (1 << ADSC);
  // Wait until ADSC bit has been cleared (conversion complete)
  while (ADCSRA & (1<<ADSC));
}

void readTrimPots(){
  // Clear MUX bits to use ADC0
  ADMUX &= ~(0b00011111);
  triggerADC();
  // Use ADC value to set motor PWM output
  OCR0A = 0b01010000 + (ADCH >> 4);

  // Set MUX bits to use ADC1
  ADMUX |= 1;
  triggerADC();
  // Use ADC value to set motor PWM output
  OCR0B = 0b01010000 + (ADCH >> 4);
}
