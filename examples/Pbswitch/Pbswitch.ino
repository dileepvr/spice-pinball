#include <Pinball.h>

int buttonpin = 10;

Pb_switch myswitch(200);    // specify debounce time ( 200 ms )



void setup() {
  
  pinMode(buttonpin, INPUT); 
  digitalWrite(buttonpin, HIGH);  // Enable internal pullup resistor

  Serial.begin(9600);

}


void loop() {

  
  if ( myswitch.pushed( digitalRead(buttonpin) ) ) {

    Serial.println("pushed");
    
  }
  
}