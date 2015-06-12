#include <Pinball.h>

int clkpin = 11;
int diopin = 12;

#define TEST_DELAY 250
#define TEST_DELAY2 50

// Declare object named myboard
Pb_scoreboard myboard(clkpin, diopin);

void setup() {

  // Display 0 without leading zeros  
  myboard.showdisplay(0);
  delay(TEST_DELAY);
  
  // Blank out display
  myboard.blankdisplay();
  delay(TEST_DELAY);

  // Display 0 with leading zeros  
  myboard.showdisplay(0, true);
  delay(TEST_DELAY);  

  myboard.blankdisplay();
  delay(TEST_DELAY);
  
}

void loop() {
 
  int ii, jj;

  // Count up without leading zeros
  for(ii = 1; ii < 10000; ii = 2*ii) {
    myboard.showdisplay(ii);
    delay(TEST_DELAY);
  }
  
  
  // Count up with leading zeros
  for(ii = 1; ii < 10000; ii = 2*ii) {
    myboard.showdisplay(ii, true);
    delay(TEST_DELAY);
  }
   
  myboard.blankdisplay();
  delay(TEST_DELAY);

  // Put decimal partition at location 2
  myboard.setpartition(2);
  myboard.blankpredisplay();
  myboard.blankpostdisplay();
  delay(TEST_DELAY);
  
  for(ii = 0; ii < 11; ii++) {
    // display ii left of partition 
     myboard.predisplay(ii);    
     for(jj = 0; jj < 11; jj++) {
       // display jj right of partition
       myboard.postdisplay(jj); 
       delay(TEST_DELAY2);
     }
  }
 

  for(ii = 10; ii >= 0; ii--) {
    // display ii left of partition   
     myboard.predisplay(ii);    
     for(jj = 0; jj < 11; jj++) {
       // display jj with leading zeros right of partition  
       myboard.postdisplay(jj, true); 
       delay(TEST_DELAY2);
     }
  }
    
  // Put decimal partition at location 1
  myboard.setpartition(1);    
  myboard.predisplay(0);
  myboard.blankpostdisplay();
  delay(TEST_DELAY);
  
  myboard.postdisplay(1); delay(TEST_DELAY);
  myboard.postdisplay(10); delay(TEST_DELAY);
  myboard.postdisplay(100); delay(TEST_DELAY);  
  myboard.predisplay(1);
  myboard.postdisplay(0,true); delay(TEST_DELAY);    
  myboard.postdisplay(100,true); delay(TEST_DELAY);
  myboard.postdisplay(10,true); delay(TEST_DELAY);
  myboard.postdisplay(1,true); delay(TEST_DELAY);        

  myboard.setpartition(3);    
  myboard.postdisplay(0);
  myboard.blankpredisplay();
  delay(TEST_DELAY);
  
  myboard.predisplay(100); delay(TEST_DELAY);
  myboard.predisplay(20); delay(TEST_DELAY);
  myboard.predisplay(3); delay(TEST_DELAY);  
  myboard.postdisplay(4);
  myboard.blankpredisplay(); delay(TEST_DELAY);    
  myboard.predisplay(3); delay(TEST_DELAY);
  myboard.predisplay(20); delay(TEST_DELAY);
  myboard.predisplay(100); delay(TEST_DELAY);      
}
