#include "Arduino.h"

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

void setup(){
  pinMode(2,OUTPUT); //attach pin 2 to vcc
  pinMode(5,OUTPUT);  //attach pin 5 GND
  digitalWrite(2, HIGH);
  Serial.begin(9600);
}

void loop(){
  long ping_read = sonicScan.sonicRead();
  Serial.println(ping_read);
  delay(100);
}


