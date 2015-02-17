#include <ros.h>                  //includes the necessary packages
//#include <std_msgs/String.h>
#include <std_msgs/Float32.h>     //since we're sending numbers now
#include <std_msgs/Int16.h>
#include <Wire.h>
#include <L3G.h>
#include <Servo.h>


ros::NodeHandle  nh;              //creates our node handle on the arduino --> I think this is similar to the ROS Master for the python code
L3G gyro;   //creates our gryo instance
Servo servo;
int clockDelay = 1;
int servoDelay = 10;
int stepSize = 1;
int delayCount = 0;
int desiredServoPosition = 90;
int servoPosition = 90;

std_msgs::Float32 value;      //initializes the message that we want to send
ros::Publisher xRotation("xRotation", &value);    //creates our publisher, with the type of message that it will send
ros::Publisher yRotation("yRotation", &value);
ros::Publisher zRotation("zRotation", &value);

void moveServo(const std_msgs::Int16 &angle)
{
  if (angle.data > 135)
  {
    desiredServoPosition = 135;
  }
  else
  {
    desiredServoPosition = (int)angle.data;
  }
}

void setStepSize(const std_msgs::Int16 &givenStepSize)
{
  if (givenStepSize.data > 15)
  {
    stepSize = 15;
  }
  else
  {
    stepSize = (int)givenStepSize.data;
  }
}

ros::Subscriber <std_msgs::Int16> speedSubscription("ServoSpeed", setStepSize);
ros::Subscriber <std_msgs::Int16> angleSubscription("ServoAngle", moveServo);

void setup()
{
  Serial.begin(9600);
  Wire.begin();
  
  servo.attach(3);
  
  if (!gyro.init())
  {
    value.data = 100000;
    while (1);
  }
  
  gyro.enableDefault();
  
  nh.initNode();        //creates a node that is the arduino
  nh.advertise(xRotation);   //makes the scope publisher of the arduino node available
  nh.advertise(yRotation);
  nh.advertise(zRotation);
  nh.subscribe(speedSubscription);
  nh.subscribe(angleSubscription);
}

void loop()
{
  if (delayCount<servoDelay/clockDelay)
  {
    delayCount++;
  }
  else if (desiredServoPosition < servoPosition)
  {
    servoPosition -= stepSize;
    servo.write(servoPosition);
    delayCount = 0;
  }
  else if (desiredServoPosition > servoPosition)
  {
    servoPosition +=stepSize;
    servo.write(servoPosition);
    delayCount = 0;
  }
  
  
  gyro.read();
  
  value.data = (float)gyro.g.x;    
  xRotation.publish(&value);  //publishes the string  
  value.data = (float)gyro.g.y;    
  yRotation.publish(&value);  //publishes the string  
  value.data = (float)gyro.g.z;    
  zRotation.publish(&value);  //publishes the string  
  nh.spinOnce();              //tells everything in ROS to check for messages I believe
  delay(clockDelay);      //wait 1 second
}
