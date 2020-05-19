#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>

#define ROBOT_SPEED_RIGHT 50
#define ROBOT_SPEED_LEFT 50
#define MAX_DESIRED_ERROR 90

// This code is designed for use with QTR-8A sensor board.
// The left sensors are connected to PD7 & PD6 (sensors 6 & 7).
// The right sensors are connected to PF5 & PF6 (sensors 2 & 3).

// Scaling factor for error signal. This value is overwritten during calibration.
uint8_t error_scaler = 1;
int16_t last_error = 0;

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

    // Scale error signal correctly.
    int16_t scaled_error = error / error_scaler;
    // Ensure that error signal does not leave desired range.
    if (scaled_error > MAX_DESIRED_ERROR){
      scaled_error = MAX_DESIRED_ERROR;
    }
    if ((scaled_error + MAX_DESIRED_ERROR) < 0){
      scaled_error = 0 - MAX_DESIRED_ERROR;
    }

    int16_t error_deriv = scaled_error - last_error;
    last_error = scaled_error;

    int16_t left_wheel_speed = ROBOT_SPEED_LEFT + scaled_error + error_deriv;
    int16_t right_wheel_speed = ROBOT_SPEED_RIGHT - scaled_error - error_deriv;

    wheelController(left_wheel_speed, right_wheel_speed);

    // Set debugging LEDs
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

    // This control loop repeats at most 50 times per second.
    _delay_ms(20);
  }
}

// Find the maximum and minimum error values under current conditions.
uint8_t calibrateSensorValues(){
  // Set up calibration variables. These values will be overwritten during the calibration phase.
  int16_t calibrated_min_error = 500;
  int16_t calibrated_max_error = -500;

  // Turn on both debug LEDs
  PORTB |= (1<<2)|(1<<1);

  for (int i = 0; i < 1500; i++){
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

  // Clear MUX bits
  ADMUX &= ~(0b00011111);
  // Set MUX bits to use ADC5 (sensor 2)
  ADMUX |= 0b00000101;
  triggerADC();
  uint8_t sen2val = ADCH;

  if (sen2val > sen3val){
    return sen2val;
  }
  else{
    return sen3val;
  }
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

  // Clear MUX bits
  ADMUX &= ~(0b00011111);
  ADCSRB &= ~(1<<5);
  // Set MUX bits to use ADC9 (sensor 7)
  ADMUX |= 0b00000001;
  ADCSRB |= (1<<5);
  triggerADC();
  uint8_t sen7val = ADCH;

  if (sen6val > sen7val){
    return sen6val;
  }
  else{
    return sen7val;
  }
}

void wheelController(int16_t left_wheel_speed, int16_t right_wheel_speed){
  // Left wheel
  if (left_wheel_speed < 0){
    // Set direction of left motor.
    PORTB |= 1;
    // Set speed of left motor.
    OCR0A = (uint8_t)(left_wheel_speed * -1);
  }
  else{
    // Set direction of left motor.
    PORTB &= ~1;
    // Set speed of left motor.
    OCR0A = (uint8_t)(left_wheel_speed);
  }

  // Right wheel
  if (right_wheel_speed < 0){
    // Set direction of right motor.
    PORTE &= ~(1<<6);
    // Set speed of right motor.
    OCR0B = (uint8_t)(right_wheel_speed * -1);
  }
  else{
    // Set direction of right motor.
    PORTE |= (1<<6);
    // Set speed of right motor.
    OCR0B = (uint8_t)(right_wheel_speed);
  }
}

// Init PWM on both motors.
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
  OCR0A = 0;  // Left motor
  OCR0B = 0;  // Right motor

  // Set direction of right motor.
  PORTE |= (1<<6);
  // Set direction of left motor.
  PORTB &= ~1;
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
