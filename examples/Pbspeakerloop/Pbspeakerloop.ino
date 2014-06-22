/*
  Program to test the Pb_speaker loop track mechanism from Pinball.h
  Attach speaker to pin 8 and ground. Attache button (which shorts to ground 
  when pressed) to pin 10.
*/

#include <Pinball.h>

// Contains notes and timing array for some looped soundtrack
// See Pinball library folder for file contents
#include <mariotracks.h>

// Initializing speaker
Pb_speaker spkr(8);

// Arbitrary event sound effect
int pointmelody[] = {NOTE_DS4, NOTE_GS4, 0};
int pointtime[] = { 125, 250, 2000};

int buttonpin = 10;
Pb_switch mysw(200);   // Using this to debounce button pushes


void setup() {

  pinMode(buttonpin, INPUT);
  digitalWrite(buttonpin, HIGH);
  
  spkr.loopstart(marionotes2, mariotimes2, 62);
  // spkr.loopstop() is also defined
}

void loop() {

  spkr.update();    // This needs to be called every loop iteration
  
  if ( mysw.pushed( digitalRead(buttonpin) ) ) {
    // This temporarily pauses loop to play sound effect
    spkr.start(pointmelody, pointtime, 3);
  }
  
}

