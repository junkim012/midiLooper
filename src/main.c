//
// Created by Jun Kim on 4/18/23.
//

#include "uart.h"
#include <avr/io.h>
#include <util/delay.h>

#define BAUD_RATE 31250
// does prescaler need to change?
#define BAUD_PRESCALER (((F_CPU / (BAUD_RATE * 16UL))) - 1)


void noteOn(int cmd, int pitch, int velocity) {
    UART_write(cmd); // 1 byte
    UART_write(pitch); // 1 byte
    UART_write(velocity); // 1 byte
}

void Initialize() {

    // write your initialization code here
    UART_init(BAUD_PRESCALER);
    // Output pins for 4 channel LEDs
    DDRB |= (1<<DDB1);
    PORTB &= ~(1<<PORTB1); // D9
    DDRB |= (1<<DDB2);
    PORTB &= ~(1<<PORTB2); // D10
    DDRB |= (1<<DDB3);
    PORTB &= ~(1<<PORTB3); // D11
    DDRB |= (1<<DDB4);
    PORTB &= ~(1<<PORTB4); // D12
    // Set input pins for 4 channels

//    TimerInit(128);
}

int main(void)
{
    Initialize();
//    int track1[200];
//    int track2[200];

//    if (PORTB )

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

    while(1)
    {

//        unsigned char data = UART_receive();
//        char string[25];
//        sprintf(string, data);
//        UART_putstring(string);

        int note2 = 0x2E;
        int channel1 = 0x90;
        int channel2 = 0x92;
        noteOn(channel1, note2, velocity);
        _delay_ms(100);

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