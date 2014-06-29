/*
  Program to test the Pb_muxin class from Pinball.h
  Attach a buttons (that shorts to ground when pressed) to 
  all 8 channels of a 4051 multiplexer.  
*/

#include <Pinball.h>


Pb_muxin mysws(2, 3, 4, 7); // (pinA, pinB, pinC, COMpin)


void setup() {

  Serial.begin(9600);

}

void loop() {

  // outputs zero in position of button pressed, rest all ones
  Serial.println(mysws.read(),BIN);
  delay(500);
}
