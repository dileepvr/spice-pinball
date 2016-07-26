/*
  This project has been released under a Modified MIT License, the text
  of which is presented in the accopanying file LICENSE.md.
  Future versions of the MIT license may not automatically apply.
*/
/*
  Program to test the Pb_speaker class from Pinball.h
  Attach speaker to pin 8 and ground.
*/

#include <Pinball.h>

// This is how you initialize speaker
// Can use multiple speakers, but only one can play at a time
Pb_speaker spkr(8);

// This is for timing
unsigned long curtime, duration1 = 3000, duration2 = 1000;

int flag = 0;

// Check notes.h in library folder for note definitions
int melody1[] = {NOTE_C4, NOTE_G3, NOTE_G3, NOTE_A3, NOTE_G3, 0, NOTE_B3, NOTE_C4};
// Timings in milliseconds
int timing1[] = { 250, 125, 125, 250, 250, 250, 250, 250};

int melody2[] = {NOTE_DS4, NOTE_GS4};
int timing2[] = {125, 250};

void setup() {

  curtime = millis(); 

}

void loop() {

  spkr.update();     // This needs to be called every loop iteration
  
  if ((flag == 0) && (millis() - curtime > duration2)) {
     spkr.start(melody1, timing1, 8);   // Last argument is length of array
     flag = 1;
     curtime = millis();
  }

  if ((flag == 1) && (millis() - curtime > duration1)) {
     flag = 0;
     spkr.start(melody2, timing2, 2);  // New melody stops already playing one if called before previous one has finished playing  
     curtime = millis();
  }

  // Also consider spkr.stop()  
  
}
