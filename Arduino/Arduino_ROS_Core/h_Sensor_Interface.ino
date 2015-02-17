//Libraries & definitions

#include <Wire.h>
#include <LSM303.h>
#include <L3G.h>
  
SonicScan sonicScan("sonicScan",9,10);    //Initialize the ultrasonic scanner to echo/trigger pin

//Initialization code
void SetupAllSensors()
{
  //GyroSetup();
  
  //Ultrasonic sensor setup
  pinMode(8,OUTPUT); //attach pin 8 to vcc
  pinMode(11,OUTPUT);  //attach pin 11 GND
  digitalWrite(8, HIGH);  //VCC on pin 8
}

float rawValue;
float resistanceVal;
int resistanceInt;
int analogValues[4] = {-1, -1, -1, -1}; 
//                  -1, -1, -1, -1, 
//                   -1, -1, -1, -1};
//
int* readAnalogIns()
{
  //Reads all the analog inputs to check 
  //Convert using resistance value
  //TODO: smarter conversion here (resistance is hard-coded below)

  rawValue = 0;
  resistanceVal = 0;
  resistanceInt = 0;

  //Collect all analog sensor values
  for(int i = 0; i < sizeof(analogValues); i++)
  {
      rawValue = analogRead('A' + (char)i);
      resistanceVal = (rawValue * (5.0 / 1023.0));
      resistanceInt = int(resistanceVal);
      analogValues[i] = resistanceInt;  
      delay(50);     //Keep an eye out for this delay. Might give speed or reading problems later
  }

  return analogValues;    //Return reference
}


//Note: this is just a convenience wrapper
long toReturn = 0;
long readSonicScanner()
{
  toReturn = sonicScan.sonicRead();
  return toReturn;
}

