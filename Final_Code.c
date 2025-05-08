#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "intrinsics.h"
#include "msp430fr2355.h"
#include <msp430.h>

volatile bool redLEDActive = false;
volatile bool greenLEDActive = false;

enum State {IDLE, HEATING}; // System states
enum State currentState = IDLE; // Initial state

// Function Declarations
void flame_Init();
int flame_detect();
void therm_Init();
int16_t therm_read();
void pot_init();
int16_t pot_read();
void mainValve_init();
void MainValve_set(uint16_t pulseWidth);
void pilotvalve_init();
void pilotvalve_on();
void pilotvalve_off();
void rgb_init();
void rgb_setColor(char Red, char Green, char Blue);
void ignitor_init();
void ignitor_on();
void ignitor_off();
void uart_init();
void delay_ms(uint16_t ms);

void flame_Init() {
    P1SEL0 |= BIT3;
    P1SEL1 |= BIT3; // Set P1.3 to ADC function
}

int flame_detect() {
    ADCCTL0 &= ~ADCENC; // Ensure ADC is not enabled while configuring
    ADCCTL0 |= ADCSHT_2 | ADCON;
    ADCCTL1 = ADCSHP;
    ADCMCTL0 = ADCINCH_3; // Channel 3 (P1.3)
    ADCCTL0 |= ADCENC | ADCSC;
    while (!(ADCIFG & ADCIFG0));
    return ADCMEM0; // Return flame sensor ADC value
}

void therm_Init() {
    P1SEL0 |= BIT6;
    P1SEL1 |= BIT6; // Set P1.6 to ADC function
}

int16_t therm_read() {
    // Configure ADC for the temperature sensor on channel 6 (P1.6)
    ADCCTL0 |= ADCSHT_2 | ADCON; // Set sample-and-hold time and turn on ADC
    ADCCTL1 = ADCSHP; // Use the sampling timer
    ADCMCTL0 = ADCINCH_6; // Select channel 6

    // Dummy conversion to allow the ADC's sampling capacitor to settle
    ADCCTL0 |= ADCENC | ADCSC;
    while (!(ADCIFG & ADCIFG0));
    ADCCTL0 &= ~ADCENC; // Disable ADC conversion enable (optional reset)

    // Actual conversion reading
    ADCCTL0 |= ADCENC | ADCSC;
    while (!(ADCIFG & ADCIFG0));
    uint16_t adc_value = ADCMEM0;
    int16_t voltage = (adc_value * 3300) / 1023;  // Convert ADC value to millivolts (assuming 3.3V)
    int16_t celsius = voltage / 10; // Convert mV to °C (10 mV per °C)
    return (celsius * 9 / 5) + 32; // Convert °C to °F
}

void pot_init() {
    P1SEL0 |= BIT5;
    P1SEL1 |= BIT5; // Set P1.5 to ADC function
}

int16_t pot_read() {
    // Configure ADC for the potentiometer on channel 5 (P1.5)
    ADCCTL0 |= ADCSHT_2 | ADCON;
    ADCCTL1 = ADCSHP;
    ADCMCTL0 = ADCINCH_5;

    // Dummy conversion to allow settling after switching channels
    ADCCTL0 |= ADCENC | ADCSC;
    while (!(ADCIFG & ADCIFG0));
    ADCCTL0 &= ~ADCENC;  // Disable the conversion enable

    // Actual conversion reading
    ADCCTL0 |= ADCENC | ADCSC;
    while (!(ADCIFG & ADCIFG0));
    uint16_t adc_value = ADCMEM0;
    return (adc_value * 100) / 1023; // Scale the output to a range of 0–100 (percentage)
}

void mainValve_init() {
    P2DIR |= BIT1; // Set P2.1 as output
    P2SEL0 |= BIT1;
    P2SEL1 &= ~BIT1;
    PM5CTL0 &= ~LOCKLPM5; // Unlock I/O

    TB1CCR0 = 20000; // PWM period
    TB1CCTL2 = OUTMOD_7; // Reset/Set mode
    TB1CTL = TBSSEL__SMCLK | MC__UP | TBCLR; // SMCLK, Up mode, clear timer
}

void MainValve_set(uint16_t pulseWidth) {
    TB1CCR2 = pulseWidth; // Set PWM duty cycle
}

void pilotvalve_init() {
    P5DIR |= BIT4; // Set P5.4 as output
    P5OUT &= ~BIT4; // Start OFF
}

void pilotvalve_on() {
    P5OUT |= BIT4;
}

void pilotvalve_off() {
    P5OUT &= ~BIT4;
}

void rgb_init() {
    P6DIR |= BIT0 | BIT1 | BIT2; // Set RGB pins as outputs
    P6SEL0 |= BIT0 | BIT1 | BIT2;
    P6SEL1 &= ~(BIT0 | BIT1 | BIT2);
    PM5CTL0 &= ~LOCKLPM5; // Unlock GPIO

    TB3CCR0 = 1000 - 1; // PWM period
    TB3CCTL1 = OUTMOD_3; TB3CCR1 = 250;
    TB3CCTL2 = OUTMOD_3; TB3CCR2 = 500;
    TB3CCTL3 = OUTMOD_3; TB3CCR3 = 500;
    TB3CTL = TBSSEL__SMCLK | MC__UP | TBCLR; // SMCLK, Up mode, clear timer
}

void rgb_setColor(char Red, char Green, char Blue) {
    TB3CCR3 = Red << 2; // Shift values to match PWM resolution
    TB3CCR2 = Green << 2;
    TB3CCR1 = Blue << 2;
}

void ignitor_init() {
    P5DIR |= BIT0;  // Set P5.0 as output
    P5OUT &= ~BIT0; // Start OFF
}

void ignitor_on() {
    P5OUT |= BIT0;
}

void ignitor_off() {
    P5OUT &= ~BIT0;
}

void uart_init() {
    P4SEL0 |= BIT3 | BIT2; // UART TX/RX pins
    P1SEL1 &= ~(BIT3 | BIT2);

    UCA1CTLW0 |= UCSWRST; // Hold in reset
    UCA1CTLW0 |= UCSSEL__SMCLK; // SMCLK as source
    UCA1BR0 = 52;  // Baud rate settings for 9600 (assuming 1MHz)
    UCA1BR1 = 0;
    UCA1MCTLW = 0x4910; // Modulation settings
    UCA1CTLW0 &= ~UCSWRST; // Release reset
}

void delay_ms(uint16_t ms) {
    while (ms--) {
        __delay_cycles(1000); / Delay ~1ms assuming 1MHz clock
    }
}

int main() {
    uart_init();
    flame_Init();
    therm_Init();
    pot_init();
    mainValve_init();
    pilotvalve_init();
    rgb_init();
    ignitor_init();

    currentState = IDLE;
    rgb_setColor(0, 255, 0); // Initial color = Green (Idle)

    while (1) {
        int16_t current_temp = therm_read(); // Read current temp
        int16_t setpoint = pot_read(); // Read desired temp
        int value = flame_detect(); // Read flame sensor

        switch (currentState) {
            case IDLE:
                MainValve_set(500); // No flow
                pilotvalve_off();
                ignitor_off();
                rgb_setColor(0, 255, 0); // Green for IDLE

                if (current_temp < 50) {
                    pilotvalve_on();
                    ignitor_on();
                    delay_ms(2000); // Wait for ignition

                    if (value > 2) {  // Flame detected
                        ignitor_off();
                        MainValve_set(1200); // Full flow
                        rgb_setColor(255, 0, 0); // Red for HEATING
                        currentState = HEATING;
                    } else {
                        pilotvalve_off();
                        ignitor_off();
                        rgb_setColor(0, 0, 255); // Blue for Failure
                        delay_ms(5000);
                        currentState = IDLE;
                    }
                }
                break;

            case HEATING:
                if (!flame_detect()) { // Flame lost
                    pilotvalve_off();
                    MainValve_set(500);
                    rgb_setColor(0, 0, 255); // Blue for Failure
                    currentState = IDLE;
                    break;
                }

                if (current_temp >= setpoint) {
                    pilotvalve_off();
                    MainValve_set(500);
                    rgb_setColor(0, 255, 0); // Green for IDLE
                    currentState = IDLE;
                }
                break;

            default:
                rgb_setColor(0, 0, 255); // Blue for unknown state
                break;
        }

        delay_ms(500); // Loop delay
    }
}
