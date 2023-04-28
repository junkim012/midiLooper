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
char String[25];

int channel;

volatile int record1 = 0;
int track1[300];
int step = 0;

void noteOn(int cmd, int pitch, int velocity) {
    UART_write(cmd); // 1 byte
    UART_write(pitch); // 1 byte
    UART_write(velocity); // 1 byte
}

ISR(PCINT0_vect) {
    if (PINB & (1 << PINB0)) {
        record1 = record1 ^ 1;

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
        UART_write(status_byte); // 1 byte
        UART_write(data_byte1); // 1 byte
        UART_write(data_byte2); // 1 byte

//        int concat = (status_byte << 16) | (data_byte1 << 8) | (data_byte2);
//        char printConcat[25];
//        sprintf(printConcat, "STEP: %d CONCAT: %X\n", step, concat);
//        track1[step] = concat;
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
        // ADC doesn't work with UART
//        int printADC[25];
//        sprintf(printADC, "ADC: %u\n", ADC);
//        UART_putstring(printADC);

        cli();
        unsigned char data = UART_receive();
        char printData[25];
        sprintf(printData, "DATA: %X\n", data);
        UART_putstring(printData);

        process_midi_data(data);
        sei();
//        if (step == 300) {
//            step = 0;
//        } else {
//            step = step + 1;
//        }

//        _delay_ms(1000);

//        int note1 = 0x48;
//        int channel1 = 0x90;
//        int channel2 = 0x92;
//        noteOn(channel1, note1, velocity);
//        _delay_ms(1000);

        // check if pin is low
//        if (!PINB & (1<<PB1)) {v
//            // turn on LED
//            UART_write("hihihi");
//        }

        // add incoming to sequence array
//        if (!(UCSR0A & (1<<RXC0))) {
//            int printData[25];
//            int hi = 0x20;
//            sprintf(printData, "%x\n", hi);
//            UART_putstring(printData);
//            int data = UART_receive();
//            sequence[step] = data; // should overflow
//            step = step + 1;
//        }



//        if (PORTB & (1<<PB1)) {
//            // just print something for indication
//            int printData[25];
//            int hi = 0x20;
//            sprintf(printData, "%x\n", hi);
//            UART_putstring(printData);
//
//            for (int i = 0; i < 300; i = i+3) {
//                noteOn(sequence[i],
//                       sequence[i+1],
//                       sequence[i+2]);
//                _delay_ms(1000);
//            }
//        }
//
//        int printData[25];
//        sprintf(printData, "%x\n", data);
//        UART_putstring(printData);
//        if (data == 0x91) {
//            UART_putstring("2");
//        }

        // play the 2d array
//        for (int i = 0; i < seqLength; i++ ) {
//            noteOn(channel1, track[0][i], velocity);
//            noteOn(channel1, track[1][i], velocity);
//            noteOn(channel1, track[2][i], velocity);
//            _delay_ms(100);
//            counter++;
//            if (i == 4) {
//                i = 0;
//            }
//        }
//        counter++;
//        if (counter == loopLength) {
//            counter = 0;
//        }
//
//        noteOn(channel2, note1, velocity);
//        noteOn(channel2, note2, velocity);
//        noteOn(channel2, note3, velocity);
//        noteOn(channel2, note4, velocity);


//        noteOn(channel3, note, velocity);

        // write to the standard 5 poles DIN cable
        // 31250 bits per second

        // Digital pin 1 -> MIDI jack pin 5
        // Midi Jack pin 2 ground
        // Midi Jack pin 4 to +5V through a 220 ohm resistor
    }
}