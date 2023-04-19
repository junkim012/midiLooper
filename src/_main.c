//#include <Arduino.h>
//
//volatile int overflow_counter = 0;
//volatile int overflow_space_counter = 0;
//volatile int overflow_target = 0;
//
//// most recent position in the MIDI track array
//int step = 0;
//// boolean that indicates playback
//int playback = 0;
//
//// whenever timer overflows
//ISR(TIMER1_OVF_vect) {
////    if (pressed) {
//    overflow_counter = overflow_counter + 1;
////    }
//    if (overflow_counter == overflow_target) {
//        // play the metronome sound
//
//        overflow_counter = 0;
//    }
//}
//
///**
// * Sets an overflow timer
// * interrupts every tick of the metronome
// * @param bpm
// */
//void TimerInit(int bpm) {
//
//    // beats per minute
//    // if 120 beats / min
//    // 120 / 60 = 2 beats / sec
//    // 0.5 sec / beat
//    // should interrupt every 0.5 seconds
//    // how to make this super accurate???
//
//    // Timer 1
//    TCCR1B &= ~(1<<CS10);
//    TCCR1B &= ~(1<<CS11);
//    TCCR1B |= (1<<CS12);
//
//    // "enable the clock using the TCCR1B register"
//    TCCR1A &= ~(1<<WGM10);
//    TCCR1A &= ~(1<<WGM11);
//    TCCR1B &= ~(1<<WGM12);
//    TCCR1B &= ~(1<<WGM13);
//
//    // Look for rising edge
//    TCCR1B |= (1<<ICES1);
//
//    // Enable input capture interrupt
//    // "Enable the interrupt in the TIMSK1 register"
//    TIMSK1 |= (1<<ICIE1);
//
//    // enable timer overflow interrupt
//    TIMSK1 |= (1<<TOIE1);   // enable timer overflow interrupt
//
//    // clear input capture flag
//    TIFR1 |= (1<<ICF1);
//
//    sei();
//}
//
//void MIDIInit() {
////    MIDI.begin(MIDI_CHANNEL_OMNI);
////    MIDI.turnThruOff();
//}
//
///**
// * Checks whether the play switch is on
// * Is this analog input pin? (check component datasheet)
// * toggles between two states of playback and pause
// */
//int playbackSwitch() {
//    // 0 if off, 1 if on
//    return 0;
//}
//
///**
// * Start listening to MIDI signals
// */
//void MIDIStart() {
//    // play switch on
//    if (playbackSwitch()) {
//        step = 0;
//    } else {
//        playback = 0;
//    }
//}
//
//void Initialize() {
//// write your initialization code here
//
//    // Output pins for 4 channel LEDs
//    DDRB |= (1<<DDB1);
//    PORTB &= ~(1<<PORTB1);
//    DDRB |= (1<<DDB2);
//    PORTB &= ~(1<<PORTB2);
//    DDRB |= (1<<DDB3);
//    PORTB &= ~(1<<PORTB3);
//    DDRB |= (1<<DDB4);
//    PORTB &= ~(1<<PORTB4);
//    // Set input pins for 4 channels
//
//    TimerInit(128);
//}
//
//void loop() {
//// write your code here
//}