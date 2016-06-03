/*
  Program to test the Pb_timedevent class from Pinball.h
  Attach a button (that shorts to ground when pressed) to pin 10.
*/


#include <Pinball.h>

// This is how you initialize a timedevent label
// The argument can be any function (defined somewhere in the program)
// that returns void and accepts an integer argument
void printval(int);
Pb_timedevent dothis(printval);

int values[] = { 2, 3, 5, 7, 11};
int timing[] = { 1000, 500, 500, 2000, 500};  // milliseconds
// The last number in the timing array doesn't matter, as this array only stores
// time till next execution

int buttonpin = 10;
Pb_switch mysw(200);   // Using Pb_switch to debounce

void setup() {

  pinMode(buttonpin, INPUT);
  digitalWrite(buttonpin, HIGH);
  Serial.begin(9600);
  
}

void loop() {

  dothis.update();      // This needs to be called every loop iteration
  
  if ( mysw.pushed( digitalRead(buttonpin)) ) {
   
    // Start timed sequence if button is pressed
    dothis.start(values, timing, 5);    // Last argument is length of sequence
    // dothis.stop() also exists
  }
  
}

void printval( int val) {
 
  Serial.println(val);
  
}
