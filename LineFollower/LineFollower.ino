#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>

#define ROBOT_SPEED_RIGHT 40
#define ROBOT_SPEED_LEFT 40
#define MAX_DESIRED_ERROR 30;

// This code is designed for use with QTR-8A sensor board.
// The left sensors are connected to PD7 & PD6 (sensors 6 & 7).
// The right sensors are connected to PF5 & PF6 (sensors 2 & 3).

// Scaling factor for error signal. This value is overwritten during calibration.
uint8_t error_scaler = 1;

int main()
{
  initADC();
  error_scaler = calibrateSensorValues();

  // Turn on IR LEDs.
  DDRB |= (1<<3);
  PORTB |= (1<<3);

  // Enable LED2 and LED3
  DDRB |= (1<<1)|(1<<2);

  initPWM();

  while (1)
  {
    uint8_t reading_right = readRightSensor();
    uint8_t reading_left = readLeftSensor();
    int16_t error = reading_right - reading_left;

    OCR0A = ROBOT_SPEED_LEFT + (error / error_scaler);
    OCR0B = ROBOT_SPEED_RIGHT - (error / error_scaler);

    if (error < 0){
      // Turn to the left
      PORTB |= (1<<1);
      PORTB &= ~(1<<2);
    }
    else if (error > 0){
      // Turn to the right
      PORTB |= (1<<2);
      PORTB &= ~(1<<1);
    }
    else{
      // Go straight
      PORTB |= (1<<2)|(1<<1);
    }

    // This control loop repeats at most 40 times per second.
    _delay_ms(25);
  }
}

// Find the maximum and minimum error values under current conditions.
uint8_t calibrateSensorValues(){
  // Set up calibration variables. These values will be overwritten during the calibration phase.
  int16_t calibrated_min_error = 500;
  int16_t calibrated_max_error = -500;

  // Turn on both debug LEDs
  PORTB |= (1<<2)|(1<<1);

  for (int i = 0; i < 2000; i++){
    uint8_t reading_right = readRightSensor();
    uint8_t reading_left = readLeftSensor();
    int16_t error = reading_right - reading_left;

    if (error > calibrated_max_error){
      calibrated_max_error = error;
    }
    if (error < calibrated_min_error){
      calibrated_min_error = error;
    }
    _delay_ms(2);
  }

  // Warn user that calibration is complete by flashing LEDs.
  for (int i = 0; i < 5; i++){
    PORTB |= (1<<2)|(1<<1);
    _delay_ms(500);
    PORTB &= ~((1<<2)|(1<<1));
    _delay_ms(500);
  }

  return calibrated_max_error/MAX_DESIRED_ERROR;
}

uint8_t readRightSensor(){
  // Clear MUX bits
  ADMUX &= ~(0b00011111);
  ADCSRB &= ~(1<<5);
  // Set MUX bits to use ADC6 (sensor 3)
  ADMUX |= 0b00000110;
  triggerADC();
  uint8_t sen3val = ADCH;
  sen3val = sen3val / 2;

  // Clear MUX bits
  ADMUX &= ~(0b00011111);
  // Set MUX bits to use ADC5 (sensor 2)
  ADMUX |= 0b00000101;
  triggerADC();
  uint8_t sen2val = ADCH;
  sen2val = sen2val / 2;

  return sen2val + sen3val;
}

uint8_t readLeftSensor(){
  // Clear MUX bits
  ADMUX &= ~(0b00011111);
  ADCSRB &= ~(1<<5);
  // Set MUX bits to use ADC10 (sensor 6)
  ADMUX |= 0b00000010;
  ADCSRB |= (1<<5);
  triggerADC();
  uint8_t sen6val = ADCH;
  sen6val = sen6val / 2;

  // Clear MUX bits
  ADMUX &= ~(0b00011111);
  ADCSRB &= ~(1<<5);
  // Set MUX bits to use ADC9 (sensor 7)
  ADMUX |= 0b00000001;
  ADCSRB |= (1<<5);
  triggerADC();
  uint8_t sen7val = ADCH;
  sen7val = sen7val / 2;

  return sen6val + sen7val;
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
  OCR0A = 150;  // Left motor
  OCR0B = 150;  // Right motor

  // Set direction of right motor. Left motor turns in forward direction by default.
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
