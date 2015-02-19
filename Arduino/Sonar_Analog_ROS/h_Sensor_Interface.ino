#include <Wire.h>

//Libraries & definitions
  
SonicScan sonicScan("sonicScan",9,10);    //Initialize the ultrasonic scanner to echo/trigger pin

//Initialization code
void SetupAllSensors()
{
  //Ultrasonic sensor setup
  pinMode(8,OUTPUT); //attach pin 8 to vcc
  pinMode(11,OUTPUT);  //attach pin 11 GND
  digitalWrite(8, HIGH);  //VCC on pin 8
  
  //TODO analog setup
}

//TODO clean these variables up, as needed
float rawValue;
float resistanceVal;
int resistanceInt;
int analogValues[analogPins];

//Reads all analog pins (as defined by analogPins defined in Sonar_Analog_ROS) and returns array of their values
//TODO unit conversion? Or just return raw values as-is?
int* readAnalogIns()
{
  //Reads all the analog inputs to check 
  //Convert using resistance value
  //TODO: smarter conversion here (resistance is hard-coded below)

  rawValue = 0;
  resistanceVal = 0;
  resistanceInt = 0;

  //Collect all analog sensor values
  for(int i = 0; i < analogPins; i++)
  {
      rawValue = analogRead(i);
      //resistanceVal = (rawValue * (5.0 / 1023.0));
      //resistanceInt = int(resistanceVal);
      //analogValues[i] = resistanceInt;  
      analogValues[i] = rawValue;
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

