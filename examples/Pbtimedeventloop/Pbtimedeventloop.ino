/*
  This project has been released under a Modified MIT License, the text
  of which is presented in the accopanying file LICENSE.md.
  Future versions of the MIT license may not automatically apply.
*/
/*
  Program to test the Pb_timedevent loopd sequence mechanism from Pinball.h
  Attach a button (that shorts to ground when pressed) to pin 10.
*/


#include <Pinball.h>

// This is how you initialize a timedevent label
// The argument can be any function (defined somewhere in the program)
// that returns void and accepts an integer argument
void printval(int);
Pb_timedevent dothis(printval);

// This is the looped sequence that will be printed
int values[] = { 2, 3, 5, 7, 11};
int timing[] = { 1000, 500, 500, 2000, 1000};  // milliseconds

// This is the interrupt event that the button will trigger
// This will pause the looped sequence to run
int interr[] = { 42, 42 };
int intert[] = { 1000, 1000 };

int buttonpin = 10;
Pb_switch mysw(200);   // Using Pb_switch to debounce

void setup() {

  pinMode(buttonpin, INPUT);
  digitalWrite(buttonpin, HIGH);
  Serial.begin(9600);
  
  dothis.loopstart(values, timing, 5);
  // dothis.loopstop() is also defined
}

void loop() {

  dothis.update();      // This needs to be called every loop iteration
  
  if ( mysw.pushed( digitalRead(buttonpin)) ) {
   
    // Start interrupt sequence if button is pressed
    dothis.start(interr, intert, 2);    // Last argument is length of sequence
    // dothis.stop() also exists
  }
  
}

void printval( int val) {
 
  Serial.println(val);
  
}
