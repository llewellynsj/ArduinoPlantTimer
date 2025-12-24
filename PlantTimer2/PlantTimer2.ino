/*
 * File: PlantTimer_v2_Internal.ino
 * Description: Version 2 of the Plant Timer using ATtiny85 Internal WDT.
 * - Replaces 'goto' logic with a State Machine.
 * - Adds Calibration to compensate for WDT drift.
 * - Adds visual feedback (blinks) on start to confirm settings.
 *
 * Microcontroller: ATtiny85
 * Author: Gemini (Refactoring original concept)
 * Date: 15 Dec 2025
 */

#include <avr/sleep.h>      // Sleep modes
#include <avr/wdt.h>        // Watchdog Timer
#include <avr/interrupt.h>  // Interrupts (ISR)
#include <avr/power.h>      // Power management

// =========================================================================
// --- USER CONFIGURATION & CALIBRATION ---
// =========================================================================

// 1. PIN DEFINITIONS
#define LED_PIN     PB0  // Pin 5 on DIP8
#define SWITCH_PIN  PB3  // Pin 2 on DIP8 (Analog Input)
#define BUTTON_PIN  PB4  // Pin 3 on DIP8

// 2. TIMING SETTINGS (Days)
#define DAYS_SHORT   1UL
#define DAYS_MEDIUM  5UL
#define DAYS_LONG    10UL

// 3. CALIBRATION (The Magic Number)
// The WDT is nominally 8.0 seconds, but often runs faster or slower.
// If your timer is finishing TOO FAST, decrease this number (e.g., 7.8).
// If your timer is finishing TOO SLOW, increase this number (e.g., 8.2).
// Formula: New_Cal = Current_Cal * (Actual_Time_Elapsed / Target_Time)
#define WDT_CALIBRATED_SECONDS  8.0 

// =========================================================================
// --- SYSTEM CONSTANTS & VARIABLES ---
// =========================================================================

// ADC Thresholds for the 3-position switch (0-1023 scale)
#define ADC_THRES_LOW   340  // Lower third
#define ADC_THRES_HIGH  680  // Upper third

// State Machine States
enum TimerState {
    STATE_STARTUP,      // Initial power on, read settings, blink LED
    STATE_SLEEPING,     // Deep sleep, counting down WDT cycles
    STATE_ALARM,        // Timer finished, LED ON, waiting for user
    STATE_RESET_WAIT    // Handling button press to reset
};

TimerState currentState = STATE_STARTUP;

// Volatile flags set by Interrupt Service Routines (ISRs)
volatile bool flag_wdt_tick = false;
volatile bool flag_button_press = false;

// Timing counters
unsigned long wake_cycles_counter = 0;
unsigned long target_wake_cycles = 0;

// Function Prototypes
void setupWDT();
void sleepDeep();
void sleepIdle();
unsigned long calculateTargetCycles(unsigned long days);
int readSwitchMode();
void blinkIndicator(int times);

// =========================================================================
// SETUP
// =========================================================================
void setup() {
    // 1. Configure Pins
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    
    pinMode(BUTTON_PIN, INPUT_PULLUP); 
    pinMode(SWITCH_PIN, INPUT); 

    // 2. Power Saving (Disable unused peripherals)
    ADCSRA &= ~(1 << ADEN); // Disable ADC
    power_adc_disable();
    
    // 3. Setup Interrupts
    // Enable Pin Change Interrupt on PB4 (Button)
    GIMSK |= (1 << PCIE);
    PCMSK |= (1 << PCINT4);

    // 4. Setup Watchdog
    setupWDT();

    // Enable Global Interrupts
    sei();
}

// =========================================================================
// MAIN LOOP (State Machine)
// =========================================================================
void loop() {
    switch (currentState) {
        
        // --- CASE: STARTUP / RESET ---
        case STATE_STARTUP: {
            // 1. Read the switch to determine duration
            int mode = readSwitchMode(); // Returns 1 (Short), 2 (Medium), or 3 (Long)
            
            // 2. Calculate how many 8-second cycles we need
            unsigned long days = 0;
            if (mode == 1) days = DAYS_SHORT;
            else if (mode == 2) days = DAYS_MEDIUM;
            else days = DAYS_LONG;
            
            target_wake_cycles = calculateTargetCycles(days);
            wake_cycles_counter = 0;

            // 3. Visual Feedback: Blink LED to confirm setting
            // 1 Blink = Short, 2 Blinks = Medium, 3 Blinks = Long
            blinkIndicator(mode);

            // 4. Reset flags and transition to sleep
            flag_button_press = false;
            flag_wdt_tick = false;
            currentState = STATE_SLEEPING;
            break;
        }

        // --- CASE: SLEEPING (COUNTDOWN) ---
        case STATE_SLEEPING: {
            // Check if button was pressed during sleep
            if (flag_button_press) {
                // Debounce check
                delay(50); 
                if (digitalRead(BUTTON_PIN) == LOW) {
                    // Valid press: Reset timer
                    blinkIndicator(1); // Short blink to acknowledge
                    currentState = STATE_STARTUP;
                }
                flag_button_press = false; // Clear flag
            }
            // Check if Watchdog woke us up
            else if (flag_wdt_tick) {
                flag_wdt_tick = false; // Clear flag
                wake_cycles_counter++;
                
                // Check if we reached the target
                if (wake_cycles_counter >= target_wake_cycles) {
                    currentState = STATE_ALARM;
                } else {
                    // Go back to sleep
                    sleepDeep();
                }
            } 
            else {
                // If we are here and no flags are set, just sleep
                sleepDeep();
            }
            break;
        }

        // --- CASE: ALARM (WATER ME!) ---
        case STATE_ALARM: {
            digitalWrite(LED_PIN, HIGH); // Turn LED ON

            // We use Idle sleep here to save power while keeping the LED on.
            // Only a button press (Interrupt) will wake us fully.
            if (flag_button_press) {
                 // Debounce
                delay(50);
                if (digitalRead(BUTTON_PIN) == LOW) {
                    // User acknowledged alarm
                    digitalWrite(LED_PIN, LOW);
                    currentState = STATE_STARTUP; // Restart cycle
                }
                flag_button_press = false;
            } else {
                sleepIdle();
            }
            break;
        }
    }
}

// =========================================================================
// HELPERS & LOGIC
// =========================================================================

// Calculates the number of WDT cycles needed for X days
unsigned long calculateTargetCycles(unsigned long days) {
    // Total seconds needed
    unsigned long totalSeconds = days * 24UL * 3600UL;
    // Divide by calibrated WDT interval
    return (unsigned long)(totalSeconds / WDT_CALIBRATED_SECONDS);
}

// Reads the analog switch (enables ADC temporarily)
int readSwitchMode() {
    power_adc_enable();
    ADCSRA |= (1 << ADEN); // Enable ADC

    // ADMUX Config: Vcc Ref, ADC3 (PB3)
    ADMUX = (0 << REFS1) | (0 << REFS0) | (0 << ADLAR) | (1 << MUX1) | (1 << MUX0);
    delay(2); // Settling time

    ADCSRA |= (1 << ADSC); // Start conversion
    while (bit_is_set(ADCSRA, ADSC)); // Wait
    int val = ADC;

    ADCSRA &= ~(1 << ADEN); // Disable ADC
    power_adc_disable();

    if (val < ADC_THRES_LOW) return 1;      // Short
    else if (val < ADC_THRES_HIGH) return 2;// Medium
    else return 3;                          // Long
}

// Blinks the LED 'times' times
void blinkIndicator(int times) {
    for (int i = 0; i < times; i++) {
        digitalWrite(LED_PIN, HIGH);
        delay(200);
        digitalWrite(LED_PIN, LOW);
        delay(200);
    }
}

// =========================================================================
// LOW LEVEL & SLEEP
// =========================================================================

void setupWDT() {
    cli();
    wdt_reset();
    // Enable WDT configuration change
    WDTCR |= (1 << WDCE) | (1 << WDE);
    // Set Interrupt Mode (WDIE) and ~8s interval (WDP3+WDP0)
    WDTCR = (1 << WDIE) | (1 << WDP3) | (1 << WDP0);
    sei();
}

void sleepDeep() {
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    sei();       // Ensure interrupts are enabled
    sleep_cpu(); // Goodnight
    sleep_disable();
}

void sleepIdle() {
    set_sleep_mode(SLEEP_MODE_IDLE); // Keeps peripherals (timers/PWM) running if needed
    sleep_enable();
    sei();
    sleep_cpu();
    sleep_disable();
}

// =========================================================================
// INTERRUPTS
// =========================================================================

// Watchdog Vector
ISR(WDT_vect) {
    flag_wdt_tick = true;
}

// Pin Change Vector (Button)
ISR(PCINT0_vect) {
    // Check if Button (PB4) is LOW
    if (!(PINB & (1 << BUTTON_PIN))) {
        flag_button_press = true;
    }
}