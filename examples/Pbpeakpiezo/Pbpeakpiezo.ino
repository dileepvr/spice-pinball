#include <Pinball.h>

// arguments: A_pin, reset_pin, threshold, peak capture time (ms), reset hold time (ms)
Pb_peakpiezo mypiezo(A0, 4, 80, 100, 150);

int Aval;

void setup() {

  mypiezo.reset();

  Serial.begin(9600);

}

void loop() {

  mypiezo.update();

  Aval = mypiezo.read();

  if(Aval > 0) {
    Serial.println(Aval);
  }
  
}
