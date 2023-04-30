//
// Created by Jun Kim on 4/18/23.
//

#include "uart.h"
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include <uart_duplex.h>

#define BAUD_RATE 31250

// does prescaler need to change?
#define BAUD_PRESCALER (((F_CPU / (BAUD_RATE * 16UL))) - 1)
//#define BAUD_PRESCALER 31
#define MIN_INTERVAL_MS 1000 // minimum interval in milliseconds between array indexing
#define MAX_INTERVAL_MS 5000 // maximum interval in milliseconds between array indexing

#define TRACK_LENGTH 20

char String[25];

int channel;

volatile int count = 0;
int targetCount = 0;

volatile int record1_armed = 0;
volatile int record1 = 0;

//int track1[20][3] = {
//        {0x93, 0x24, 0x40} ,
//        {0x00, 0x00, 0x00} ,
//        {0x00, 0x00, 0x00},
//        {0x83, 0x24, 0x40},
//        {0x00, 0x00, 0x00} ,
//        {0x00, 0x00, 0x00},
//        {0x93, 0x24, 0x40} ,
//        {0x00, 0x00, 0x00} ,
//        {0x00, 0x00, 0x00}
//};
int track1[TRACK_LENGTH][3];
int step = 0;

// timing stuff
// TNCT0 counts from 0 to OCR0A and move onto the next
// step variable increments everytime OCR0A is reached

void noteOn(int cmd, int pitch, int velocity) {
    UART_write(cmd); // 1 byte
    UART_write(pitch); // 1 byte
    UART_write(velocity); // 1 byte
}

ISR(PCINT0_vect) {
    if (PINB & (1 << PINB0)) {
        record1_armed = 1;

        PORTD |= (1 << PORTD2);
        PORTD &= ~(1 << PORTD3);
        PORTD &= ~(1 << PORTD5);
        PORTD &= ~(1 << PORTD4);
        channel = 0x91; // switch channel to 2
    } else if (PINB & (1 << PINB1)) {
        PORTD |= (1 << PORTD3);
        PORTD &= ~(1 << PORTD5);
        PORTD &= ~(1 << PORTD2);
        PORTD &= ~(1 << PORTD4);
        channel = 0x91; // switch channel to 2
    } else if (PINB & (1 << PINB2)) {
        PORTD |= (1 << PORTD4);
        PORTD &= ~(1 << PORTD5);
        PORTD &= ~(1 << PORTD3);
        PORTD &= ~(1 << PORTD2);
        channel = 0x92; // switch channel to 3
    }
}


void Initialize() {

    cli(); // disable global interrupts;

    // write your initialization code here
    UART_init(BAUD_PRESCALER);
    // Output pins for 4 channel LEDs

    DDRB &= ~(1<<DDB1); // set to input pin
//    PORTB |= (1<<PORTB1); // pull D9 high

//    DDRB |= (1<<DDB2);
//    PORTB &= ~(1<<PORTB2); // D10
//    DDRB |= (1<<DDB3);
//    PORTB &= ~(1<<PORTB3); // D11
//    DDRB |= (1<<DDB4);
//    PORTB &= ~(1<<PORTB4); // D12
    // Set input pins for 4 channels

    // set up LEDs as output
    DDRD |= (1 << DDD2); // channel 1 LED
//    PORTD |= (1 << PORTD2);
    DDRD |= (1 << DDD3); // channel 2 LED
//    PORTD |= (1 << PORTD3);
    DDRD |= (1 << DDD4); // channel 3 LED
    DDRB |= (1 << DDB4); // set up tempo LED

    // set up buttons as input
    DDRB &= ~(1 << DDB0); // channel 1 button (pin change interrupt)
    PORTB |= (1<<PORTB0); // enable pull up resistor on PB2
    PCMSK0 |= (1<<PCINT0); // enable trigger for PCINT0

    PCICR |= (1<<PCIE0); //enable PCINT1 pin change interrupt
    DDRB &= ~(1 << DDB1); // channel 2 button (pin change interrupt)
    PORTB |= (1<<PORTB1); // enable pull up resistor on PB1
    PCMSK0 |= (1<<PCINT1); // enable trigger for PCINT1
    DDRB &= ~(1 << DDB2); // channel 3 button (pin change interrupt)
    PORTB |= (1<<PORTB2); // enable pull up resistor on PB2
    PCMSK0 |= (1<<PCINT2); // enable trigger for PCINT2

    // TIMER for BPM
    // CTC on OCR1A
    OCR1A = 65535; // 16.384 miliseconds
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1 = 0;
//    OCR1A = MIN_INTERVAL_MS * (F_CPU / 1000) / 64 - 1; // Set Timer1 compare value to minimum interval
    TCCR1B |= (1 << WGM12); // Set Timer1 to CTC mode
    TCCR1B |= (1 << CS11) | (1 << CS10); // Set Timer1 prescaler to 64
    TIMSK1 |= (1 << OCIE1A); // Enable Timer1 compare interrupt


    // POTENTIOMETER
    // Reading in potentiometer
    PRR &= ~(1 << PRADC);
    ADMUX |= (1 << REFS0);
    ADMUX &= ~(1 << REFS1);
    // Set the ADC Clock div by 128
    ADCSRA |= (1 << ADPS0);
    ADCSRA |= (1 << ADPS1);
    ADCSRA |= (1 << ADPS2);
    // Select channel 0
    ADMUX &= ~(1 << MUX0);
    ADMUX &= ~(1 << MUX1);
    ADMUX &= ~(1 << MUX2);
    ADMUX &= ~(1 << MUX3);
    // Set to auto trigger
    ADCSRA |= (1 << ADATE);
    // Set to free running
    ADCSRB &= ~(1 << ADTS0);
    ADCSRB &= ~(1 << ADTS1);
    ADCSRB &= ~(1 << ADTS2);
    // Disable digital input buffer on ADC pin
    DIDR0 |= (1 << ADC0D);
    // Enable ADC
    ADCSRA |= (1 << ADEN);
    // ADC interrupt
    // start conversion
    ADCSRA |= (1 << ADSC);

    sei(); // enable global interrupts this stops midi from working
}

void process_midi_message(uint8_t status_byte, uint8_t data_byte1, uint8_t data_byte2) {

    uint8_t message_type = status_byte & 0xF0;

    if (message_type >> 4 == 9 || message_type >> 4 == 8) {
        // note on message
        char printData[25];
//        sprintf(printData, "ON: %X %X %X\n", status_byte, data_byte1, data_byte2);
//        UART_putstring(printData);
        UART_send(status_byte); // 1 byte
        UART_send(data_byte1); // 1 byte
        UART_send(data_byte2); // 1 byte

//        char printConcat[25];
//        sprintf(printConcat, "STEP: %d CONCAT: %X\n", step, concat);
//        track1[step] = concat;
        if (record1 == 1) {
//            char printRecord[25];
//            sprintf(printRecord, "RECORDING NOTE %X at STEP %d\n", data_byte1, step);
//            UART_putstring(printRecord);
            track1[step][0] = status_byte;
            track1[step][1] = data_byte1;
            track1[step][2] = data_byte2;
        }
//        if (record2 == 2) {
//
//        }

    }
}

void process_midi_data(uint8_t midi_byte) {

    static uint8_t status_byte = 0;
    static uint8_t data_byte1 = 0;
    static uint8_t data_byte2 = 0;
    static uint8_t bytes_received = 0;

    if ((midi_byte >> 4) == 8 || (midi_byte >> 4 == 9)) {
        // This is a status byte
        status_byte = midi_byte;
        bytes_received = 1;
    }
    else {
        // This is a data byte
        if (bytes_received == 1) {
            data_byte1 = midi_byte;
            bytes_received = 2;
        }
        else if (bytes_received == 2) {
            data_byte2 = midi_byte;
            bytes_received = 3;
        }
        else {
            // Error: received unexpected data byte
            bytes_received = 0;
        }
    }

    if (bytes_received == 3) {
        // We have received a complete MIDI message
        process_midi_message(status_byte, data_byte1, data_byte2);
        bytes_received = 0;
    }

}

//void sendMidiSignal(int midi) {
//    // check if valid midi signal
//    if (midi[0] >> 4 == 9 || midi[0] >> 4 == 8) {
//        UART_send(midi >> 16 & 0xFF);
//        UART_send(midi >> 8 & 0xFF);
//        UART_send(midi & 0xFF);
//    }
//}

ISR(TIMER1_COMPA_vect) {

    if (count == 1) {
        // Timer1 compare interrupt service routin
        if (step == 0 && record1_armed) {
//            char printArm[25];
//            sprintf(printArm, "RECORD START");
//            UART_putstring(printArm);
            record1 = 1;
        } else if (step == 19 && record1 == 1) {
//            char printStop[25];
//            sprintf(printStop, "RECORDS STOP");
//            UART_putstring(printStop);
            record1 = 0;
            record1_armed = 0;
        }


//        char printStep[50];
//        sprintf(printStep, "STEP: %d, ADC: %u, OCR1A: %u record1_armed: %d record1: %d \n\n",
//                step, ADC, OCR1A, record1_armed, record1);
//        UART_putstring(printStep);

        // Do something with the midi signal at the current step, for example:
//    char printNote[25];
//    sprintf(printNote, "NOTE: %X, %X, %X\n",
//            track1[step][0], track1[step][1], track1[step][2]);
//    UART_putstring(printNote);

        if (track1[step][0] >> 4 == 9 || track1[step][0] >> 4 == 8) {
            UART_send(track1[step][0]);
            UART_send(track1[step][1]);
            UART_send(track1[step][2]);
        }
        step++; // Increment the step index
        if (step >= TRACK_LENGTH) {
            step = 0; // Reset step index when it reaches the end of the array
        }
        count = 0;
    } else {
        count++;
    }
}

long map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int main(void)
{
    Initialize();
    int loopLength = 10;
    int counter = 0;
    int seqLength = 5;
    // 0x2E Bb
    // 0x30 C 3
    // 0x23 B 1
    // 0x21 A 1
    // 0x34 E 3
//    int track[3][5] = {
//            {0x2E, 0x30, 0x23, 0x21, 0x34},
//            {0x30, 0x30, 0x23, 0x21, 0x34},
//            {0x23, 0x23, 0x23, 0x21, 0x34}
//    };
    int velocity = 0x45;
    int sequence[300];
    while(1)
    {
//        int printTrack[25];
//        sprintf(printTrack, "TRACK: %X\n", track1[0]);
//        UART_putstring(printTrack);
        // ADC doesn't work with UART
//        int printADC[25];
//        sprintf(printADC, "ADC: %u\n", ADC);
//        UART_putstring(printADC);

//        int interval = map(ADC, 0, 1023, MIN_INTERVAL_MS, MAX_INTERVAL_MS);
//        int printInterval[25];
//        sprintf(printInterval, "INTERVAL: %d\n", interval);
//        UART_putstring(printInterval);

        // 10000 * 16M / 1000 / 64 -1 = 2.5 million
        // 1000 => 250K
//        targetCount = interval / ((OCR1A + 1) * 64 / F_CPU * 1000);
//        int printTarget[25];
//        sprintf(printTarget, "INTERVAL: %d\n", printTarget);
//        UART_putstring(printTarget);

//        OCR1A = interval * (F_CPU / 1000) / 64 - 1;

//        int printOCR1A[25];
//        sprintf(printOCR1A,  "OCR1A: %u\n", OCR1A);
//        UART_putstring(printOCR1A);

        unsigned char data = UART_receive();

//        char printData[25];
//        sprintf(printData, "DATA: %X\n", data);
//        UART_putstring(printData);

        process_midi_data(data);
    }
}