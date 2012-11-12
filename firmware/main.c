/*

GarageParkingAssistant

Hardware connections
out
PC0 - Red LED
PC1 - Yellow LED
PC2 - Green LED
PB2 - SRF04 Init

in
PB1 - User button
PB0 - SRF04 Echo

*/

#include <stdlib.h>
#include <avr/io.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <avr/interrupt.h>

// defines
#define LEDREDPORT              PORTC
#define LEDREDDDR               DDRC
#define LEDREDPIN               PC0
#define LEDYELLOWPORT           PORTC
#define LEDYELLOWDDR            DDRC
#define LEDYELLOWPIN            PC1
#define LEDGREENPORT            PORTC
#define LEDGREENDDR             DDRC
#define LEDGREENPIN             PC2
#define SONARINITPORT           PORTB
#define SONARINITDDR            DDRB
#define SONARINITPIN            PB2
#define SONARECHOPORT           PORTB
#define SONARECHODDR            DDRB
#define SONARECHOINPUT          PINB
#define SONARECHOPIN            PB0
#define BUTTONPORT              PORTB
#define BUTTONDDR               DDRB
#define BUTTONINPUT             PINB
#define BUTTONPIN               PB1
#define EEPROM_OSCCAL           32
#define EEPROM_POSITION         0
#define OFF                     0
#define ON                      1
#define RED                     0
#define YELLOW                  1
#define GREEN                   2
#define CONVINCHES              158
#define SAMECOUNT               600

volatile static uint16_t zones[4];
volatile static uint16_t buttonCounter = 0;
volatile static uint16_t sameCounter = 0;
volatile static int16_t inches = 0;
volatile static int16_t inchesLast = 0;
volatile static int16_t inchesLastDifference = 0;
volatile static int16_t inchesDifference = 0;
volatile static uint16_t rawDist = 0;
volatile static uint16_t risingEdge, fallingEdge, pulseWidth;
volatile static uint8_t measuring = 0;

void initHardware() {
// read and set internal oscillator calibration byte
    uint8_t osccal = eeprom_read_byte((uint8_t*)EEPROM_OSCCAL);
    OSCCAL = osccal;

// LED pin directions set to output
    LEDREDDDR |= _BV(LEDREDPIN);
    LEDYELLOWDDR |= _BV(LEDYELLOWPIN);
    LEDGREENDDR |= _BV(LEDGREENPIN);

// LED pins set HIGH (LED off)
    LEDREDPORT |= _BV(LEDREDPIN);
    LEDYELLOWPORT |= _BV(LEDYELLOWPIN);
    LEDGREENPORT |= _BV(LEDGREENPIN);

// Sonar init pin set to output
    SONARINITDDR |= _BV(SONARINITPIN);

// Sonar echo pin set to input
    SONARECHODDR &= ~_BV(SONARECHOPIN);

// Button pin set to input with pullup enabled
    BUTTONDDR &= ~_BV(BUTTONPIN);
    BUTTONPORT |= _BV(BUTTONPIN);

    TCNT1 = 0;

   //SETS PRESCALER ON 1
   TCCR1B |= (1<<CS10);

   // Enable Input noise canceller and capture time on rising edge
   TCCR1B |=  (1<<ICES1) | (1<<ICNC1) ;

   TIMSK |= (1<<TICIE1); //Enable Input Capture Interrupt

       sei();
}

// convenience function for setting LEDs
void setLED(uint8_t led, uint8_t val) {
    switch(led) {
        case RED:
            if(val == OFF) {
                LEDREDPORT |= _BV(LEDREDPIN);
            } else {
                LEDREDPORT &= ~(_BV(LEDREDPIN));
            }
            break;
        case YELLOW:
            if(val == OFF) {
                LEDYELLOWPORT |= _BV(LEDYELLOWPIN);
            } else {
                LEDYELLOWPORT &= ~(_BV(LEDYELLOWPIN));
            }
            break;
        case GREEN:
            if(val == OFF) {
                LEDGREENPORT |= _BV(LEDGREENPIN);
            } else {
                LEDGREENPORT &= ~(_BV(LEDGREENPIN));
            }
            break;
    }
}

void pause(uint16_t ms) {
    uint16_t i;
    for(i = 0; i < ms; i++) {
        _delay_ms(1);
    }
}

uint8_t buttonPressed() {
    return !(BUTTONINPUT & _BV(BUTTONPIN));
}

void loadPositions() {
    uint8_t i ;
    for(i = 0; i < 4; i++) {
        zones[i] = eeprom_read_word((uint16_t*)(EEPROM_POSITION + (i * 2)));
    }
}

void lampTest(uint16_t wait) {
    setLED(RED, ON);
    setLED(YELLOW, ON);
    setLED(GREEN, ON);
    pause(wait);
    setLED(RED, OFF);
    setLED(YELLOW, OFF);
    setLED(GREEN, OFF);
}

void showError(uint8_t err) {
    uint8_t i;
    for(;;) {
        for(i = 0; i < err; i++) {
            setLED(RED, ON);
            pause(250);
            setLED(RED, OFF);
            pause(250);
        }
        pause(2000);
    }
}

void getSonar() {
    rawDist = 0;

    SONARINITPORT &= ~(_BV(SONARINITPIN));
    _delay_us(10);
    SONARINITPORT |= _BV(SONARINITPIN);

    while(measuring == 1) {}
    pause(10);
    inches = pulseWidth / CONVINCHES;
}

void programMode() {
// flash all LEDs 3 times
    uint8_t j, k;
    for(k = 0; k < 3; k++) {
        lampTest(750);
        pause(1000);
    }

// Zone 4 (GREEN)
    j = 0;
    do {
        getSonar();
        if(j++ < 3) {
            setLED(GREEN, ON);
        } else if(j > 4) {
            j = 0;
        } else {
            setLED(GREEN, OFF);
        }
    } while(!buttonPressed());

    zones[3] = inches;
    eeprom_write_word((uint16_t*)(6), zones[3]);

    for(k = 0; k < 3; k++) {
        lampTest(750);
        pause(500);
    }

// Zone 3 (YELLOW)
    j = 0;
    do {
        getSonar();
        if(j++ < 3) {
            setLED(YELLOW, ON);
        } else if(j > 4) {
            j = 0;
        } else {
            setLED(YELLOW, OFF);
        }
    } while(!buttonPressed());

    zones[2] = inches;
    eeprom_write_word((uint16_t*)(4), zones[2]);

    for(k = 0; k < 3; k++) {
        lampTest(750);
        pause(500);
    }

// Zone 2 (RED)
    j = 0;
    do {
        getSonar();
        if(j++ < 3) {
            setLED(RED, ON);
        } else if(j > 4) {
            j = 0;
        } else {
            setLED(RED, OFF);
        }
    } while(!buttonPressed());

    zones[1] = inches;
    eeprom_write_word((uint16_t*)(2), zones[1]);

    for(k = 0; k < 3; k++) {
        lampTest(750);
        pause(500);
    }

// Zone 1 (Flashing RED)
    zones[0] = zones[1] - 6;
    if(zones[0] < 6) {
        zones[0] = 6;
    }

    eeprom_write_word((uint16_t*)(0), zones[0]);
}

ISR(TIMER1_CAPT_vect)
{
   // If ICP pin is set, there was a rising edge else if its low there must have been a falling edge /
   if (bit_is_set(PINB,0))
   {
      risingEdge = ICR1;
      TCCR1B &= ~(1<<ICES1); //Capture now on falling edge
      measuring = 1;
   }
   else
   {
      fallingEdge = ICR1;
      TCCR1B |= (1<<ICES1);//Capture now on rising edge
      pulseWidth = fallingEdge - risingEdge;
      measuring = 0;
    }
}

int main() {
    uint8_t i = 0;
// initialize hardware
    initHardware();

// lamp test, turn LEDs on for 1 second
    lampTest(1000);
    pause(1000);

// load saved positions from eeprom
    loadPositions();
// if any position is 0xff, assume not set in eeprom
    if((zones[0] == 0xffff) || (zones[1] == 0xffff) || (zones[2] == 0xffff) || (zones[3] == 0xffff)) {
        programMode();
    }

    for(;;) {
        PORTB ^= _BV(PB4);
// check if button is pressed
        if(buttonPressed()) {
            buttonCounter++;
        } else {
            buttonCounter = 0;
        }

        if(buttonCounter > 10) {
// program mode
            programMode();
        } else {
// normal mode
            getSonar();

            // check for no change in distance
            inchesDifference = abs(inches - inchesLast);

            int16_t difference = abs(inchesDifference - inchesLastDifference);

            if(difference < 6) {
                sameCounter++;
            } else {
                sameCounter = 0;
            }

            inchesLast = inches;
            inchesLastDifference = inchesDifference;

            // if distance hasn't changed for a while, turn off all LEDs and try again in one second
            if(sameCounter > SAMECOUNT) {
                setLED(RED, OFF);
                setLED(YELLOW, OFF);
                setLED(GREEN, OFF);
                pause(1000);
            } else {
                if(inches < zones[0]) {
                    setLED(YELLOW, OFF);
                    setLED(GREEN, OFF);
                    i++;
                    if(i < 3) {
                        setLED(RED, ON);
                    } else if(i > 4) {
                        i = 0;
                    } else {
                        setLED(RED, OFF);
                    }
            // Stop
                } else if((inches < zones[1]) && (inches >= zones[0])) {
                    setLED(RED, ON);
                    setLED(YELLOW, OFF);
                    setLED(GREEN, OFF);
            // Slow Down
                } else if((inches < zones[2]) && (inches >= zones[1])) {
                    setLED(RED, OFF);
                    setLED(YELLOW, ON);
                    setLED(GREEN, OFF);
            // Approach
                } else if((inches < zones[3]) && (inches >= zones[2])) {
                    setLED(RED, OFF);
                    setLED(YELLOW, OFF);
                    setLED(GREEN, ON);
            // Dead Zone
                } else {
                    setLED(RED, OFF);
                    setLED(YELLOW, OFF);
                    setLED(GREEN, OFF);
                }

                pause(200);
            }
        }
    }

    return 0;
}
