#include <Pinball.h>

// These should be analog pins to use the variable speed control.
int controlpin1 = 5; 
int controlpin2 = 6;

// Declare object named motor1
Pb_motor motor1(controlpin1, controlpin2);

void setup() {
  // The motor should already be stopped, but lets make sure.
  motor1.stop(); 
}


void loop() {

  // Start the motor moving forward at a speed of 100/255.
  motor1.forward(100);
  delay(250);

  // Start the motor moving backward at a speed of 100/255.
  motor1.back(100);
  delay(250);

  // Let the motor coast (friction will still slow it down!)
  motor1.coast();
  delay(250);

  // Start the motor moving forward at half the previous speed (50/255)
  motor1.forward(50);
  delay(250);

  // Start the motor moving backward at half the previous speed (50/255)
  motor1.back(50);
  delay(250);

  // Force the motor to stop/brake.
  motor1.stop();
  delay(250);

}
