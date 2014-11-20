//***************************************************
//Boris Leg Demonstration:
//Attempt at first leg prototype for senior design
//demonstration
//Date: 11/17/2014
//***************************************************

#include <Servo.h>

//Servo objects for knee and hip of leg
Servo kneeservo;
Servo hipservo;

//Variable to store the position of each of the servos
int kneepos;
int hippos;

void setup()
{
  kneeservo.attach(5);
  hipservo.attach(6);
}

void loop()
{
  hipservo.write(180);
  delay(50);
  kneeservo.write(0);
  delay(500);
  hipservo.write(90);
  delay(50);
  kneeservo.write(90);
  delay(500);
  hipservo.write(0);
  delay(50);
  kneeservo.write(180);
  delay(500);
  hipservo.write(90);
  delay(50);
  kneeservo.write(90);
  delay(500);
  
  /*
  for(hippos = 0; hippos < 90; hippos += 5)  // goes from 0 degrees to 180 degrees 
  {                                  // in steps of 1 degree 
    for(kneepos = 0; kneepos < 90; kneepos += 5)
    {
      hipservo.write(hippos);
      kneeservo.write(kneepos);              // tell servo to go to position in variable 'pos' 
      delay(100);                       // waits 15ms for the servo to reach the position 
    }
  } 
  for(hippos = 90; hippos >= 1; hippos -= 5)     // goes from 180 degrees to 0 degrees 
  {
    for(kneepos = 90; kneepos >= 1; kneepos -=  5)
    {    
      hipservo.write(hippos);    
      kneeservo.write(kneepos);              // tell servo to go to position in variable 'pos' 
      delay(100);                       // waits 15ms for the servo to reach the position
    }
  } */
}
