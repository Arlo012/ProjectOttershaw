#include <ros.h>
#include <std_msgs/Int32.h>
//#include <ottershaw_masta/Analog.h>

//Analog pins for vibration sensor on each leg
# define analogPins 8
const int legs[] = {A0, A1, A2, A3, A4, A5, A6, A7};  //Is this possible? Took idea from Knock.ino example

//Threshold constants for the piezo vibration sensor
//TODO: Define these values through testing of sensor
#define standingThreshold 0
#define movingThreshold 1
#define collisionThreshold 2

//LED pins
//could incorporate legs on black widow symbol
//lighting up when on ground

//Begin main class declaration
ros::NodeHandle nh;

//Publisher
std_msgs:: Int32 int_msg;
//ottershaw_masta::Analog analog_msg;

ros::Publisher piezo("piezo", &int_msg);

void setup()
{
  //pinMode(legLEDs, OUTPUT); //There will be more than one
  
  //ROS node initialization
  nh.initNode();
  nh.advertise(piezo);
  nh.spinOnce();
  
}

void loop()
{
  for(int i=0; i < (analogPins-1); i++)
  {
    int_msg.data = detectLegStatus(legs[i]);
    piezo.publish(&int_msg);
    nh.spinOnce();
  }
}

int detectLegStatus(int legPin)
{
  if(analogRead(legPin) < standingThreshold)
  {
    return 0;
  }
  else if(analogRead(legPin) < movingThreshold)
  {
    return 1;
  }
  else if(analogRead(legPin) > collisionThreshold)
  {
    return -1;
  }
}
