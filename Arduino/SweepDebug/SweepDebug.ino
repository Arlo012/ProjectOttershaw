// Sweep
// by BARRAGAN <http://barraganstudio.com> 
// This example code is in the public domain.


#include <Servo.h> 
 
Servo servo1;
Servo servo2;
Servo servo3;
 
int pos = 0;    // variable to store the servo position 
 
void setup() 
{ 
  servo1.attach(5);  // attaches the servo on pin 9 to the servo object 
  servo2.attach(10);  // attaches the servo on pin 9 to the servo object 
  servo3.attach(11);  // attaches the servo on pin 9 to the servo object 
} 
 
 
void loop() 
{                                // in steps of 1 degree 
  servo1.write(0);              // tell servo to go to position in variable 'pos' 
  delay(1250);                 // waits 15ms for the servo to reach the position 
  servo1.write(180);              // tell servo to go to position in variable 'pos' 
  delay(1250);    
} 
