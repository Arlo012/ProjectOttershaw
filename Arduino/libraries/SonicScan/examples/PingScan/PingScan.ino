
/*
  Test sketch for SonicScan Library.
  This code takes readings from the sensor and outputs the values
  in the console.
*/

#include <SonicScan.h>
#include "Arduino.h"

  int myPins[] = {3,4};  //trig and echo pins on ultrasonic sensor

//constructor for Sonic Scan class
//SonicScan sonicScan(String("ping"),myPins[0], myPins[1]);

void setup()
{
  pinMode(2,OUTPUT); //attach pin 2 to vcc
  pinMode(5,OUTPUT);  //attach pin 5 GND
 Serial.begin(9600);
}

void loop()
{
  //sonicScan.sonicRead();
}

