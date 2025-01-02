//working code copy - 29.12.24

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/sleep.h>


volatile uint8_t seconds = 0;
volatile uint8_t minutes = 30;
volatile uint8_t hours = 12;
volatile uint8_t brightness_stage = 0; 	   // Tracks the current brightness stage (0-3)
volatile uint8_t pwm_hold_timer = 0;       // Tracks how long the brightness button is held
volatile uint8_t pwm_button_held = 0;      // Flag indicating if the button is currently held
volatile uint8_t min_hold_timer = 0;
volatile uint8_t min_button_held = 0;
volatile uint8_t hour_hold_timer = 0;
volatile uint8_t hour_button_held = 0;

const uint8_t brightness_levels[] = {254, 248, 156, 0}; // Define brightness levels

void setup_timer2() {
    // Activate asynchronous mode for Timer2 with external clock
    ASSR |= (1 << AS2);

    // Set Timer2 to Normal mode
    TCCR2A = 0;                     // Normal mode
    TCCR2B = (1 << CS22) | (1 << CS20); // Prescaler = 128

    // Enable Timer2 Overflow interrupt
    TIMSK2 = (1 << TOIE2);

    // Wait for asynchronous clock stabilization
    while (ASSR & ((1 << TCR2AUB) | (1 << TCR2BUB))) {
        // Wait until Timer2 registers are updated
    }
}

void setup_timer1() {
    // Configure Timer1 for PWM on PB1 and PB2
    TCCR1A = (1 << WGM10) | (1 << COM1A1) | (1 << COM1B1); // 8-bit Fast PWM, Clear on Compare Match
    TCCR1B = (1 << WGM12) | (1 << CS11); // Prescaler = 8

    OCR1A = brightness_levels[brightness_stage]; // Set initial brightness
    OCR1B = brightness_levels[brightness_stage]; // Set initial brightness
}

void setup_ports() {
    // Hours (PORTC: PC0 to PC4) and Minutes (PORTD: PD0 to PD5) as outputs
    DDRC |= 0x1F;  // PC0 to PC4 for hours
    DDRD |= 0x3F;  // PD0 to PD5 for minutes

    // Turn off LEDs initially
    PORTC &= ~0x1F; // Hours LEDs off
    PORTD &= ~0x3F; // Minutes LEDs off

    // PB1 and PB2 as outputs for PWM
    DDRB |= (1 << PB1) | (1 << PB2);

    // Buttons (PB0, PD6, and PD7) as inputs with Pull-Up resistors
    DDRB &= ~(1 << PB0); // PB0 as input
    DDRD &= ~((1 << PD6) | (1 << PD7)); // PD6 and PD7 as inputs
    PORTB |= (1 << PB0); // Activate Pull-Up resistor for PB0
    PORTD |= (1 << PD6) | (1 << PD7); // Activate Pull-Up resistors for PD6 and PD7
}

void setup_interrupts() {
    // Enable external interrupts on PB0, PD6, and PD7
    PCICR |= (1 << PCIE0) | (1 << PCIE2); // Pin-Change Interrupt for PORTB and PORTD
    PCMSK0 |= (1 << PCINT0);              // PB0 as interrupt pin
    PCMSK2 |= (1 << PCINT22) | (1 << PCINT23); // PD6 and PD7 as interrupt pins
}

void update_display() {
    // Update hours on PORTC (PC0 to PC4)
    PORTC = (PORTC & ~0x1F) | (hours & 0x1F);

    // Update minutes on PORTD (PD0 to PD5)
    PORTD = (PORTD & ~0x3F) | (minutes & 0x3F);
}

ISR(TIMER2_OVF_vect) {
    // Increment seconds
    seconds++;

    // Handle button hold timing for brightness button (PD7)
    if (pwm_button_held) {
        pwm_hold_timer++;
        if (pwm_hold_timer >= 2) { // ~2 seconds hold
            OCR1A = 255;
            OCR1B = 255;
            pwm_button_held = 0; // Stop further timing
	    brightness_stage = (brightness_stage - 1) % 4;
	    //sleep_mode();
        }
    }
    if (min_button_held) {
        min_hold_timer++;
        if (min_hold_timer >= 1) { // ~1 seconds hold
	    minutes--;
	    seconds = 0;
            min_button_held = 0; // Stop further timing
        }
    }
    if (hour_button_held) {
        hour_hold_timer++;
        if (hour_hold_timer >= 1) { // ~1 seconds hold
	    hours--;
            hour_button_held = 0; // Stop further timing
        }
    }


    if (seconds >= 60) {
        seconds = 0;
        minutes++;
        if (minutes >= 60) {
            minutes = 0;
            hours++;
            if (hours >= 24) {
                hours = 0; // Reset after 24 hours
            }
        }
    }

    update_display(); // Update LED display
}

ISR(PCINT0_vect) {
    // Handle PB0 button (Increment minutes)
    if (OCR1A != 255 && !(PINB & (1 << PB0))) { // Button pressed
        _delay_ms(5); // Simple debounce delay
        if (!(PINB & (1 << PB0))) { // Confirm button is still pressed
            min_button_held = 1; // Start hold timing
            min_hold_timer = 0;  // Reset hold timer
        }
    } else { // Button released
        _delay_ms(5); // Simple debounce delay
        if ((PINB & (1 << PB0))) { // Confirm button is still pressed
	    if (min_button_held && min_hold_timer < 1) { // Normal button press
                minutes++;
		seconds = 0;
                if (minutes >= 60) {
                    minutes = 0;
                    hours++;
                    if (hours >= 24) {
                        hours = 0;
                    }
                }
	    }
	    min_button_held = 0; // Reset hold flag
            update_display(); // Update immediately
        }
    }
    //_delay_ms(15);
}

ISR(PCINT2_vect) {
    // Handle PD6 button (Increment hours)
    if (OCR1A != 255 && !(PIND & (1 << PD6))) { // Button pressed
        _delay_ms(5); // Simple debounce delay
        if (!(PIND & (1 << PD6))) { // Confirm button is still pressed
            hour_button_held = 1; // Start hold timing
            hour_hold_timer = 0;  // Reset hold timer
        }
    } else { // Button released
        _delay_ms(5); // Simple debounce delay
        if ((PIND & (1 << PD6))) { // Confirm button is still pressed
	    if (hour_button_held && hour_hold_timer < 1) { // Normal button press
                hours++;
                if (hours >= 24) {
                    hours = 0;
                }
	    }
	    hour_button_held = 0; // Reset hold flag
            update_display(); // Update immediately
        }
    }

    // Handle PD7 button (Brightness control)
    if (!(PIND & (1 << PD7))) { // Button pressed
        _delay_ms(5); // Simple debounce delay
        if (!(PIND & (1 << PD7))) { // Confirm button is still pressed
            pwm_button_held = 1; // Start hold timing
            pwm_hold_timer = 0;  // Reset hold timer
        }
    } else { // Button released
        _delay_ms(5); // Debounce delay
        if ((PIND & (1 << PD7))) { // Confirm button is still released
            if (pwm_button_held && pwm_hold_timer < 2) { // Normal button press
                brightness_stage = (brightness_stage + 1) % 4;
                OCR1A = brightness_levels[brightness_stage];
                OCR1B = brightness_levels[brightness_stage];
            }
            pwm_button_held = 0; // Reset hold flag
        }
    }  
    //_delay_ms(15);  
}

void enter_sleep() {
    if(OCR1A == 255 && OCR1B == 255) {
        set_sleep_mode(SLEEP_MODE_PWR_SAVE); // Set Power-Down mode
        sleep_enable();                      // Enable sleep mode
        sleep_cpu();                         // Put the CPU to sleep
    } else {
	set_sleep_mode(SLEEP_MODE_IDLE); // Set Idle mode
        sleep_enable();                      // Enable sleep mode
        sleep_cpu();                         // Put the CPU to sleep	
    }
    // CPU wakes up here after an interrupt
    sleep_disable(); // Disable sleep mode after wake-up
}

int main() {
    setup_ports();        // Configure LED and button pins
    setup_timer2();       // Configure Timer2 for timekeeping
    setup_timer1();       // Configure Timer1 for PWM on PB1 and PB2
    setup_interrupts();   // Set up interrupts for the buttons

    sei();  // Enable global interrupts

    while (1) {
        asm("nop");
	enter_sleep(); // Enter sleep mode and wait for interrupt
        // CPU resumes here after wake-up
        _delay_ms(5); // Optional: Debounce or handle wake-up
    }

    return 0;
}
