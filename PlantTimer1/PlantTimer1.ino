/*
 * File: PlantTimer1.ino
 * Description: Prototype code for a personal project using an ATtiny85.
 * This code is intended to control an LED and a button. The LED turns on after
 * a certain period and stays on until the button is pressed. The device uses
 * sleep modes to conserve power.
 * 
 * Microcontroller: ATtiny85
 * Author: Llewellyn Sims Johns
 * Date: 12 Jan 2025
 */

#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/power.h>

#define LEDPIN 0 // Define LED pin as PB0 (digital pin 0)
#define BUTTONPIN 4 // Define button pin as PB4 (digital pin 4)
#define SWITCHPIN 3 // Define switch pin as PB3 (analog pin 3)

#define WDT_INTERVAL_TENTHS 85 // Adjusted WDT interval in tenths of a second

volatile bool watchdog_wake = false;
volatile unsigned long wake_count = 0;
volatile bool button_pressed = false;
volatile bool first_power_on = true;

void setup() {
  // Disable ADC to save power
  ADCSRA &= ~(1 << ADEN);

  pinMode(LEDPIN, OUTPUT);
  pinMode(BUTTONPIN, INPUT_PULLUP); // Set button pin as input with internal pull-up resistor
  pinMode(SWITCHPIN, INPUT); // Set switch pin as input

  // Enable pin change interrupt on BUTTONPIN
  GIMSK |= (1 << PCIE); // Enable pin change interrupts
  PCMSK |= (1 << BUTTONPIN); // Enable pin change interrupt for BUTTONPIN

  // Setup Watchdog Timer
  setupWatchdogTimer();
}

void loop() {
  if (first_power_on) {
    first_power_on = false;
    digitalWrite(LEDPIN, HIGH);

    // Wait for the button to be pressed
    while (!button_pressed) {
      set_sleep_mode(SLEEP_MODE_IDLE);
      sleep_enable();
      sleep_mode();
      sleep_disable();
    }

    // Turn off the LED
    digitalWrite(LEDPIN, LOW);

    // Reset the button press flag
    button_pressed = false;
  }

  // If we've reached the wake_count threshold
  if (wake_count >= getWakeCountThreshold()) {
    // Turn on the LED
    digitalWrite(LEDPIN, HIGH);

    // Wait for the button to be pressed
    while (!button_pressed) {
      // Enter idle sleep mode
      // breatheLED(); // Call the breatheLED function
      set_sleep_mode(SLEEP_MODE_IDLE);
      sleep_enable();
      sleep_mode();
      // The program will continue from here after waking up
      sleep_disable();
    }

    // Turn off the LED
    digitalWrite(LEDPIN, LOW);

    // Reset the button press flag
    button_pressed = false;
  }

  // Go to sleep
  goToSleep();
}

// Setup the Watchdog Timer to wake up every 8 seconds
void setupWatchdogTimer() {
  cli(); // Disable interrupts

  // Set the Watchdog Timer to 8 seconds
  wdt_reset();
  WDTCR |= (1 << WDCE) | (1 << WDE);
  WDTCR = (1 << WDP3) | (1 << WDP0); // 8 seconds
  WDTCR |= (1 << WDIE); // Enable Watchdog Timer interrupt

  sei(); // Enable interrupts
}

void goToSleep() {
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();

  // Go to sleep
  sleep_mode();

  // The program will continue from here after waking up
  sleep_disable(); // First thing to do after waking up
}

// Watchdog Timer interrupt service routine
ISR(WDT_vect) {
  watchdog_wake = true;
  wake_count++;
}

// Pin change interrupt service routine
ISR(PCINT0_vect) {
    if (digitalRead(BUTTONPIN) == LOW) {
      button_pressed = true;
      wake_count = 0; // Reset the wake count
    }
}


// Function to get the wake count threshold based on the switch position
unsigned long getWakeCountThreshold() {
  // Enable ADC for reading the switch pin
  ADCSRA |= (1 << ADEN);
  delay(1); // Allow ADC to stabilize
  int switchValue = analogRead(SWITCHPIN);

  // Disable ADC after reading to save power
  ADCSRA &= ~(1 << ADEN);

  if (switchValue < 341) {
    return ((86400 * 10) / WDT_INTERVAL_TENTHS); // 1 hour
  } else if (switchValue < 682) {
    return ((60 * 10) / WDT_INTERVAL_TENTHS); // 1 minute
  } else if (switchValue <= 1023) {
    return ((8 * 10) / WDT_INTERVAL_TENTHS); // 8 seconds
  } else {
    return ((60 * 10) / WDT_INTERVAL_TENTHS); // Default to 1 minute
  }
}

void breatheLED() {
  static const uint8_t sineWave[64] = {
    140, 152, 165, 176, 188, 198, 208, 218, 226, 234, 240, 245, 250, 253, 254, 255,
    254, 253, 250, 245, 240, 234, 226, 218, 208, 198, 188, 176, 165, 152, 140, 128,
    115, 103, 90, 79, 67, 57, 47, 37, 29, 21, 15, 10, 5, 2, 1, 0,
    1, 2, 5, 10, 15, 21, 29, 37, 47, 57, 67, 79, 90, 103, 115, 128
  };

  static uint8_t index = 0;
  static unsigned long lastUpdate = 0;
  const unsigned int delayTime = 60;

  if (millis() - lastUpdate >= delayTime) {
    analogWrite(LEDPIN, sineWave[index]);
    index = (index + 1) % 64; // Loop through the sine wave
    lastUpdate = millis();
  }
}
