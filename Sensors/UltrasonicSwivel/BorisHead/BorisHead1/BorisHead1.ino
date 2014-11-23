#include <Servo.h>

#include "Arduino.h"

Servo myservo;  // create servo object to control a servo 
                // a maximum of eight servo objects can be created 
 
int pos = 0;    // variable to store the servo position 
 
class SonicScan
{
  
  private:
    int _trigPin;
    int _echoPin;
    
    void trigPulse(){
      //Initialize the sensor by sending a pulse to the trig pin
      //The ultrasonic sensor is triggered by a high pulse > 2 microseconds
      digitalWrite(_trigPin,LOW); //send low pulse to ensure clean high pulse
      delayMicroseconds(2);
      digitalWrite(_trigPin,HIGH);
      delayMicroseconds(5);
      digitalWrite(_trigPin,LOW);
    }
    
    long msToCm(long microseconds){
      // The speed of sound is 340 m/s or 29 microseconds per centimeter.
      // The ping travels out and back, so to find the distance of the
      // object we take half of the distance travelled.
      return microseconds / 29 / 2;
    }
    
  public:
    String sonicID;
    
    SonicScan(String sonic_ID, int trigPin, int echoPin){
      pinMode(trigPin, OUTPUT);  //Trig pin initialized as output
      pinMode(echoPin, INPUT);  //Echo pin initialized as input
      sonicID = sonic_ID;  //Assigning the given ID to the class instance
      _trigPin = trigPin;
      _echoPin = echoPin;
    }
    
    long sonicRead(){
      trigPulse();
      //Returns distance reading of ultrasonic sensor in centimeters. 
      long duration, cm;
      
      //Duration in time from sending the ping to the reception of the echo off of an object
      duration = pulseIn(_echoPin, HIGH);
  
      //Unit convertion from duration to distance in centimeters
      cm = msToCm(duration);
            
      return cm;
    }
};

SonicScan sonicScan("sonicScan",3,4);

void setup()
{
  pinMode(2,OUTPUT); //attach pin 2 to vcc
  digitalWrite(2, HIGH);
  pinMode(1, OUTPUT);  //LED pin
  //myservo.attach(9);  // attaches the servo on pin 9 to the servo object 
  Serial.begin(9600);
  //delay(200);
}

void loop()
{
  blinkAndRead();
  delay(500);
  /*
  for(pos = 0; pos < 90; pos += 20)  // goes from 0 degrees to 90 degrees 
  {                                  // in steps of 1 degree 
    //myservo.write(pos);    // tell servo to go to position in variable 'pos'
    blinkAndRead();
    delay(150);                       // waits 15ms for the servo to reach the position 180
  } 
  for(pos = 90; pos>=1; pos-=20)     // goes from 90 degrees to 0 degrees 
  {                                
    //myservo.write(pos);              // tell servo to go to position in variable 'pos' 
    blinkAndRead();
    delay(150);                       // waits 15ms for the servo to reach the position 
  }
  */
}

void blinkers(int duration)
{
  //control the duration of the blink
  //for proximity sensor
  digitalWrite(1, LOW);
  delay(duration);
  digitalWrite(1, HIGH);
  delay(duration);
}

void blinkAndRead()
{
  //function that blinks led and reads data from ping sensor
  long ping_read = sonicScan.sonicRead();
  Serial.println(ping_read);
  if(ping_read <= 10)
  {
    blinkers(100);
  }
  else if((ping_read >= 10) && (ping_read <= 50))
  {
    blinkers(200);
  }
  else if((ping_read >= 50) && (ping_read <= 100))
  {
    blinkers(500);
  }
  else if(ping_read >= 100)
  {
    blinkers(1000);
  }
  else
  {
    digitalWrite(1, LOW);
  }
}

