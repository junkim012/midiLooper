
Channels.h
- `track[] channels`: The channels array keeps a list of tracks.
  - The length of the channels array is limited by microcontroller storage.
- `struct track`: Each track struct has a sequence array. 
- `int[] sequence`: Sequence array is the list of MIDI notes that will be sent to the computer to output.

Other variables we track:
- `int step`: keeps track of the latest position in the sequence array that we should be appending 
the new MIDI note to.
  - should increment based on how frequently we are listening to the MIDI input
- `boolean playback`: whether we should be sending MIDI back to the keyboard or not (0 if no, 1 if yes)
  - we should choose whether we want playback/pause for every single channel or not
- `int selectedChannel`: modified upon reading from the digitalInput connected to each channel switch
  - if channel 1 is selected, equals 1; if 2 is selected, equals 2, etc.

How MIDI Notes are being read: [discuss with TA?]
- Is there a way for a C function to passively listen to MIDI input and be triggered
  only when the MIDI note is received from the keyboard?
- Need c MIDI Library (a lot of c++ but less c)
  - Arduino's MIDI library reads 
    - very helpful guide: https://www.instructables.com/Send-and-Receive-MIDI-with-Arduino/
  - https://github.com/PortMidi/portmidi
  - https://portmedia.sourceforge.net/portmidi/

When MIDI notes are read, the function 
1. checks which channel is selected
2. if on record mode, appends the input note 


The reset channel switch just empties the corresponding sequence array to zeroes. 

UX Points 
- Switch Behavior 
  - When player presses record on a channel, it should turn playback off 

Other outstanding questions (good to ask TA)
- When we plug in the MIDI socket, how does the Uno recognize it? 
  - is it a builtin AVR library function? 
  - think the MIDI library handles this 
- can the 4 MIDI channels we have combined into one when sending to the computer? 
- if not how does it send 4 channels at once to the computer? 


Links
- MIDI smf
  - https://github.com/stump/libsmf
- Glib
  - https://docs.gtk.org/glib/
  - brew install meson ninja 