/*
  Boris the Spider Ultrasonic Sensor Library: SonicReader.cpp
  by: Project Ottershaw Group 8
  Oct-23-2014
  
  Defines SonicScan construct and the functions for trigPulse, sonicRead and getPins
*/

#include "Arduino.h"
#include "Ottershaw_SonicScan.h"

//Class constructor with ID and initialized pins trig (0) and echo (1)
SonicScan::SonicScan(String sonic_ID, int pins[])
{
  pinMode(pins[0], OUTPUT);  //Trig pin initialized as output
  pinMode(pins[1], INPUT);  //Echo pin initialized as input
  sonicID = sonic_ID;  //Assigning the given ID to the class instance
  _pins[] = pins[];
}

long sonicRead::sonicRead()
{
  trigPulse();
  //Returns distance reading of ultrasonic sensor in centimeters. 
  long duration, cm;

  //Duration in time from sending the ping to the reception of the echo off of an object
  duration = pulseIn(_pins[0], HIGH);
  
  //Unit convertion from duration to distance in centimeters
  cm = microsecondsToCentimeters(duration);
  delay(100);
  
  return cm;
}

void trigPulse::trigPulse()
{
  //Initialize the sensor by sending a pulse to the trig pin
  //The ultrasonic sensor is triggered by a high pulse > 2 microseconds
  digitalWrite(_pins[0],LOW); //send low pulse to ensure clean high pulse
  delayMicroseconds(2);
  digitalWrite(_pins[0],HIGH);
  delayMicroseconds(5);
  digitalWrite(_pins[0],LOW);
}

long microsecondsToCentimeters(long microseconds)
{
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}


