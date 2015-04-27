#include <ros.h>
#include <std_msgs/String.h>
//#include <ottershaw_masta/Analog.h>

//Analog pins for vibration sensor on each leg: uncomment the appropriate list based on your board
//#define analogPins 8  //Arduino Mega analog pins
#define analogPins 6  //Arduino Uno analog pins
//const int legs[] = {A0, A1, A2, A3, A4, A5, A6, A7};  //Analog pins for Arduino Mega
const int legs[] = {A0, A1, A2, A3, A4, A5};  //Analog pins for Arduino Uno

//LED pins
//could incorporate legs on black widow symbol
//lighting up when on ground

//ROS DECLARATIONS
ros::NodeHandle nh;
//Publisher
std_msgs:: String str_msg;
//ottershaw_masta::Analog analog_msg;
ros::Publisher piezo("piezo", &str_msg);

String vibrationMagnitudes = "";

void setup()
{
  //pinMode(legLEDs, OUTPUT); //There will be more than one
  //Serial.begin(9600);
  
  //ROS node initialization
  nh.initNode();
  nh.advertise(piezo);
  nh.spinOnce();
  
}

void loop()
{
  vibrationMagnitudes = "";
  for(int i = 0; i < analogPins; i++)
  {
    vibrationMagnitudes += analogRead(legs[i]);  
    vibrationMagnitudes += " ";
  }
  
  char charBuf[60];
  vibrationMagnitudes.toCharArray(charBuf, 60);
  str_msg.data = charBuf;
  piezo.publish(&str_msg);
  nh.spinOnce();
  delay(10);  //debugging
}

/*Used for processing on Arduino side by defining states (standing, moving, or collission) 
with 1, 0, and -1 based on threshold values. This will be done on python side instead.
*/
//int detectLegStatus(int legPin)
//{
//  int legValue = analogRead(legPin);
//  //Serial.println(legValue);  //debugging
//  if ((legValue < standingMax) && (legValue > -1))
//  {
//    return 0;
//  }
//  else if((legValue < movingMax) && (legValue > standingMax))
//  {
//    return 1;
//  }
//  else if(legValue > movingMax)
//  {
//    return -1;
//  }
//  else
//  {
//    return 2;
//  }
//}
