#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>

volatile bool watchdog_wake = false;
volatile unsigned long wake_count = 0;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  // Setup Watchdog Timer
  setupWatchdogTimer();
}

void loop() {
  // If we've reached 10,800 wakeups (approx. 24 hours)
  if (wake_count >= 5) {
    // Perform your daily task
    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);

    // Reset the counter
    wake_count = 0;
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

  // Go to sleep, waiting for Watchdog Timer to wake us up
  sleep_cpu();

  // Waking up here, after sleep
  sleep_disable();
}

ISR(WDT_vect) {
  // This function is called when the Watchdog Timer wakes the MCU
  watchdog_wake = true;
  wake_count++; // Increase the wake count
  wdt_reset();  // Reset the Watchdog Timer
}

