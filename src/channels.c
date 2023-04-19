////
//// Created by Jun Kim on 4/7/23.
////
//
//#include "channels.h"
//
//// initiate all tracks with initTrack
//void initChannel(track *c) {
//    for (int i = 0; i < CHANNELS_MAX; i++) {
//        initTrack(c[i]);
//    }
//}
//// initiate midi track with all zeroes
//void initTrack(track t) {
//    for (int i =0; i < SEQUENCE_LENGTH_MAX; i++) {
//        t.sequence[i] = 0;
//    }
//}
//
////void play() {
////    MIDI.sendNoteOn()
////}