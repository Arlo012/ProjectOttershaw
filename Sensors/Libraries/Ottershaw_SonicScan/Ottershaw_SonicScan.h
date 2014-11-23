/*
  Boris the Spider Ultrasonic Sensor Library: SonicReader
  by: Project Ottershaw Group 8
  Oct-23-2014

  Defining a class for the ultrasonic sensor which gives
  an identifyer for the sensor, the pins in which it is
  connected, and a function to read the data.
*/

#ifndef Ottershaw_SonicScan_h
#define Ottershaw_SonicScan_h

#include "Arduino.h"

class SonicScan
{
  public:
    String sonicID; //Identifier for Ultrasonic sensor
                     //Future ability: have more than one sensor
                     
    SonicScan(String sonic_ID, int pins[]);
    long sonicRead();			//Raw data read in cm
    
  private:
    int _pins[]; //Trig (_pins[0]) and Echo (_pins[1]) pin number array
    void trigPulse();	//send ping to initiate data read
};

#endif
