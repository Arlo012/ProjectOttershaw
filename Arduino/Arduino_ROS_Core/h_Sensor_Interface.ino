//Libraries & definitions

#include <SonicScan.h>
#include <Wire.h>
#include <LSM303.h>
#include <L3G.h>
  
//Analog sensor pins to read
int analogToRead[5] = {0, 1, 999, 999, 999}; //Make 999 if not in use
SonicScan sonicScan("sonicScan",9,10);    //Initialize the ultrasonic scanner to echo/trigger pin


//Initialization code
void SetupAllSensors()
{
  GyroSetup();
}
/*

//Data collection and processing functions  
String readGyroValues()
{
  //Get gyro reading here
  //Concatenate all 3 gyro axial values
  //in one CSV string
  
  String gyroCsvValues = "";  //String to be returned after convertion to a char array.
  
  //Three axial values: x, y , z
  char gyroXVal[5] = {};  
  char gyroYVal[5] = {};
  char gyroZVal[5] = {};
  
  //Convert values to char arrays
  itoa((int)ToDeg(roll), gyroXVal,10);
  itoa((int)ToDeg(pitch), gyroYVal,10);
  itoa((int)ToDeg(yaw), gyroZVal,10);

  char gyroValue[25] = {};  //TODO Change to publish angles instead of raw data
  for(int i=0; i<5;i++)
  {
    //Write all three gyro values within the same for loop
    //by allocating space in the char array
    //gyroValue[i] = gyroXVal[i];
    //gyroValue[i+5] = gyroYVal[i+5];
    //gyroValue[i+10] = gyroZVal[i+10];
  }
  String debug = "";
  return debug;
  
}
*/


float rawValue;
float resistanceVal;
int resistanceInt;
String analogResistances;

String readAnalogIns()
{
  //Reads all the analog inputs to check for values, 
  //concatenates resistances comma-separated,
  //and ignores inputs without sensors

  rawValue = 0;
  resistanceVal = 0;
  resistanceInt = 0;
  analogResistances = "";

  //Collect all analog sensor values
  //and place them in a string comma separated
  for(int i = 0; i < sizeof(analogToRead); i++)
  {
    if(analogToRead[i] != 999)
    {
      rawValue = analogRead('A' + (char)analogToRead[i]);
      resistanceVal = (rawValue * (5.0 / 1023.0));
      resistanceInt = int(resistanceVal);
      delay(50);     //Keep an eye out for this delay. Might give speed or reading problems later
      
      analogResistances += String(resistanceInt) + ","; //concatenate comma-separated resistances
    }
  }
  
  //remove extra comma at the end of string and add an end-of-string character for easier processing
  analogResistances = analogResistances.substring(0, analogResistances.length() - 1); 
  analogResistances += "!";


  return analogResistances;    //Return reference

}


String readSonicScanner()
{
  String toReturn = String(sonicScan.sonicRead());
  return toReturn;
}

