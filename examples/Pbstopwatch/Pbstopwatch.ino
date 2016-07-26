/*
  This project has been released under a Modified MIT License, the text
  of which is presented in the accopanying file LICENSE.md.
  Future versions of the MIT license may not automatically apply.
*/
#include <Pinball.h>

Pb_stopwatch mywatch;

unsigned long time;

void setup() {
  
  Serial.begin(9600);
  
}

void loop() {

  mywatch.start();
  Serial.println("Timer started");
  delay(1432);     // arbitrary delay
  mywatch.stop();  // optional
  time = mywatch.time();
  Serial.print("Time elapsed = ");
  Serial.println(time, DEC);
  
}