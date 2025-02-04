/*
Horse LED blanket

Christian Worton

ATTINY85 1MHz Internal clock

PINS:

1 RST               8 VCC
2 (PB4)             7 (PB2) BUTTON IN (EXTERNAL PULLUP)
3 (PB3) PWR LED     6 (PB1) PWM OUT LED STRIP 2
4 GND               5 (PB0) PWM OUT LED STRIP 1

*/

//#include "avr/io.h"
//#include "avr/interrupt.h"

#define F_CPU 1000000  // 1MHz

#define DEBUG 1

#define LED_PIN_1 0  // strip one
#define LED_PIN_2 1  // strip 2
#define LED_PIN_3 3  // power led
#define BTN_PIN 2    // push button

#define BTN_PRESSED 0  // button grounds the pin
#define BTN_NOT_PRESSED 1

#define BUTTON_MODE_OFF 0   // 0% duty cycle
#define BUTTON_MODE_LOW 1   //
#define BUTTON_MODE_MID 2   //
#define BUTTON_MODE_HIGH 3  // 100% duty cycle
#define BUTTON_MODE_WAVE 4  // triangle wave of LED strip brightness
#define BUTTON_MODE_MAX 4

#define WAVE_DELAY 400  // number of loops before updating wave index

#define PWR_LED_FLASH_SPEED 10
#define PWR_LED_FLASH_ON 64
#define PWR_LED_FLASH_OFF 0

volatile uint8_t button_mode = 0;   // set by pin-change interrupt
uint8_t shadow_button_mode = 0xFF;  // copied from button_mode during loop (maybe not needed)

// selectable brightness levels for LED strip
uint8_t pwm_levels[] = { 60, 130, 254 };

// triangle wave for led strip
uint8_t wave_index;
uint8_t wave_value;
uint16_t wave_counter;

volatile uint8_t pwr_led_state;
volatile uint16_t pwr_led_counter;

void init() {
  pinMode(LED_PIN_1, OUTPUT);  // LED strip 1
  pinMode(LED_PIN_2, OUTPUT);  // LED strip 2
  pinMode(BTN_PIN, INPUT);
  pinMode(LED_PIN_3, OUTPUT);  // POWER LED
  pinMode(4, INPUT_PULLUP);    // unused and tied high

  // shut down unused modules to preserve battery life:
  // bit 3 = Timer 1  (used for power LED flash)
  // bit 2 = Timer 0  (used for PWM of LEDs)
  // bit 1 = USI      (off)
  // bit 0 = ADC      (off)
  PRR = 0b00000011;

  GIMSK |= (1 << PCIE);  // enable pin change interrupt
  PCMSK = 0b00000100;    // enable pc interrupt on PB2 (pin 7 = push button)

  TCCR0A = 0b00000011;  // Fast PWM, Normal port operation, OC0A/OC0B disconnect
  TCCR0B = 0b00000010;  // PWM timer0 clock/8 (Resulting in 480Hz on 1MHz ???)
  
  TCCR1 = 0b00001100;   // timer1 is simple counter and running at clock/2048 (power led flash)
  TIMSK |= (1<<TOIE1);  // enable timer1 overflow interrupt

  SREG |= 0b10000000;  // enable global interrupts

  button_mode = BUTTON_MODE_LOW;
  shadow_button_mode = button_mode;
  analogWrite(LED_PIN_1, pwm_levels[shadow_button_mode]);
  analogWrite(LED_PIN_2, pwm_levels[shadow_button_mode]);

  pwr_led_state = 0;

  wave_counter = WAVE_DELAY;
  wave_index = 0;
}

ISR(PCINT0_vect) {
  if (digitalRead(BTN_PIN) == BTN_PRESSED) {
    if (button_mode >= BUTTON_MODE_MAX)
      button_mode = 0;
    else
      button_mode++;
  }
}

ISR(TIMER1_OVF_vect) {
  pwr_led_state=!pwr_led_state;
  digitalWrite(LED_PIN_3,pwr_led_state);
}

int main() {

  init();

  while (1) {
    // button has been pressed
    if (shadow_button_mode != button_mode) {
      shadow_button_mode = button_mode;
      if (shadow_button_mode == BUTTON_MODE_OFF) {
        analogWrite(LED_PIN_1, 0);
        analogWrite(LED_PIN_2, 0);
        // separated out as a special case incase we need
        // to do something extra in "off" state
      } else {
        analogWrite(LED_PIN_1, pwm_levels[shadow_button_mode - 1]);
        analogWrite(LED_PIN_2, pwm_levels[shadow_button_mode - 1]);
      }
    }

    // are we in wave mode and need to process the wave PWM?
    if (shadow_button_mode == BUTTON_MODE_WAVE) {
      if (--wave_counter == 0) {
        wave_counter = WAVE_DELAY;
        // 8-bit value that wraps around
        wave_index++;
        // the 256 value index range is split
        // in lower and upper half by checking the high bit.
        // this gives us a wave_value that is a triangle wave 0->127->0
        if (wave_index & 0x80) {
          wave_value = 0xFF - wave_index;
        } else {
          wave_value = wave_index;
        }
        // the wave value is multiplied by 2 to give full brightness
        analogWrite(LED_PIN_1, wave_value << 1);
        analogWrite(LED_PIN_2, (0xFF - wave_value) << 1);  // bit flipped
      }
    }
  }
  return 0;
}

