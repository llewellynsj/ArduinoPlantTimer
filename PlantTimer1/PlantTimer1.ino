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

#define LEDPIN 0 // Define LED pin as PB0 (digital pin 0)
#define BUTTONPIN 4 // Define button pin as PB4 (digital pin 4)
#define SWITCHPIN 3 // Define switch pin as PB3 (analog pin 3)

volatile bool watchdog_wake = false;
volatile unsigned long wake_count = 0;
volatile bool button_pressed = false;

void setup() {
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
  // If we've reached the wake_count threshold
  if (wake_count >= getWakeCountThreshold()) {
    // Turn on the LED
    digitalWrite(LEDPIN, HIGH);

    // Wait for the button to be pressed
    while (!button_pressed) {
      // Enter idle sleep mode
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
  int switchValue = analogRead(SWITCHPIN);
  if (switchValue < 341) { // Corresponds to 200K resistor
    return 60 / 8; // 5 seconds (assuming watchdog timer interval is 8 seconds)
  } else if (switchValue < 682) { // Corresponds to 1M resistor
    return 60 / 8; // 1 minute
  } else if (switchValue <= 1023) { // Corresponds to 5M resistor
    return 60 / 8; // 10 minutes
  } else {
    // Default case if reading fails or is out of range
    return 60 / 8; // Default to 1 minute
  }
}
