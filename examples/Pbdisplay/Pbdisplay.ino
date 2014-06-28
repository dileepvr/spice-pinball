#include <Pinball.h>

int clkpin = 12;      // Clock pin
int latchpin = 11;    // Latch pin
int datapin = 10;     // Data pin

// Declare object named disp 
Pb_display disp(datapin, clkpin, latchpin);


void setup() {
  // Write a 0 to the display.
  disp.print_number(0);
  delay(250);
}


void loop() {

  // Count to 100
  int i=0;
  for( i=0; i<100; i++ ){
    disp.print_number(i);
    delay(250);
  }  
   
}
