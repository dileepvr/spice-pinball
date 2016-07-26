/*
  This project has been released under a Modified MIT License, the text
  of which is presented in the accopanying file LICENSE.md.
  Future versions of the MIT license may not automatically apply.
*/
/*
  Program to test the Pb_speaker beat track mechanism from Pinball.h
  Attach speaker to pin 8 and ground. Attache button (which shorts to ground 
  when pressed) to pin 10.
*/

#include <Pinball.h>

// Contains notes and timing array for some looped soundtrack
// See Pinball library folder for file contents
#include <mariotracks.h>

// funky beat time needs to be an integer instead of an array
// This is meant to save space so redundant single value arrays are avoided
int funkybeattime = 100;

// Initializing speaker
Pb_speaker spkr(8);

int buttonpin = 10;
Pb_switch mysw(200);   // Using this to debounce button pushes


void setup() {

  pinMode(buttonpin, INPUT);
  digitalWrite(buttonpin, HIGH);
  
  spkr.loopstart(marionotes2, mariotimes2, 61);
  // spkr.loopstop() is also defined
}

void loop() {

  spkr.update();    // This needs to be called every loop iteration
  
  if ( mysw.pushed( digitalRead(buttonpin) ) ) {
    // This temporarily pauses loop to play sound effect, see mariotracks.h for definitions
    spkr.startbeat(funkynotes, funkybeattime, funkylength);
    // spkr.beatstop() is also defined
  }
  
}
