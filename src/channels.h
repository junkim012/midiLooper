//
// Created by Jun Kim on 4/7/23.
//

#define SEQUENCE_LENGTH_MAX 128
#define CHANNELS_MAX 4
// Channels struct include 4 channels with each of its
// specific MIDI sequences

// Each track is an array of MIDI values
typedef struct Track{
    int sequence[SEQUENCE_LENGTH_MAX];
} track;

// Channels is an array of tracks
track channels[CHANNELS_MAX];

void initChannels(int *c);
void initTrack(track t);

// operations on Track
// MIDI notes are a byte
/**
 * appends MIDI note to a track
 * @param t track struct
 * @param step index in the sequence array
 * @param note the MIDI note being added to track
 */
void addMidi(track *t, int step, unsigned char note) {
    t->sequence[step] = note;
}