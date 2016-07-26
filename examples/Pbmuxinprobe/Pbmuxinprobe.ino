/*
  This project has been released under a Modified MIT License, the text
  of which is presented in the accopanying file LICENSE.md.
  Future versions of the MIT license may not automatically apply.
*/
/*
  Program to test the Pb_muxin class from Pinball.h
  Attach a buttons (that shorts to ground when pressed) to 
  all 8 channels of a 4051 multiplexer.  
*/

#include <Pinball.h>

Pb_muxin mysws(2, 3, 4, 7); // (pinA, pinB, pinC, COMpin)

int swnum;

void setup() {

  Serial.begin(9600);
  swnum = 0;
}

void loop() {

  // Probe channel number swnum
  if ( !mysws.probe(swnum) ) { 
    Serial.print("Button press-held : ");
    Serial.println(swnum);
  }
  
  if (++swnum > 7) { swnum = 0; }
  
}
