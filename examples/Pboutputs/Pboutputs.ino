/*
  This project has been released under a Modified MIT License, the text
  of which is presented in the accopanying file LICENSE.md.
  Future versions of the MIT license may not automatically apply.
*/
#include <Pinball.h>

// Connect two 74HC595s' in series
// Connect LEDs with 1k resistors to their outputs

int clkpin = 12;      // Clock pin
int latchpin = 11;    // Latch pin
int datapin = 10;     // Data pin
int numreg = 2;       // Number of shift registers in series

// Declare object named shregs 
Pb_outputs shregs(datapin, clkpin, latchpin, numreg);

byte serdata[2];


void setup() {

 serdata[0] = 0b00000000;
 serdata[1] = 0b00000000;  // Use hex if thats easier

 shregs.update(serdata);   // Shifting out the array
 delay(250);

}


void loop() {
 
 serdata[0] = 0b01010101;
 serdata[1] = 0b10101010;  // Use hex if thats easier

 shregs.update(serdata);
 delay(250);

 serdata[0] = 0b10101010;
 serdata[1] = 0b01010101;  // Use hex if thats easier

 shregs.update(serdata);
 delay(250);

   
}

