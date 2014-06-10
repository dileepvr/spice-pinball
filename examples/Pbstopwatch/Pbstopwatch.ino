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