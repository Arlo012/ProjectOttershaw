#include <ros.h>                  //includes the necessary packages
#include <Servo.h>
#include <std_msgs/String.h>
#include <ottershaw_masta/Servo.h>

//#define arraySize 24
const int arraySize=24;
Servo servos[arraySize];
int desiredAngle[arraySize];
int currentAngle[arraySize];
int stepSize[arraySize];


ros::NodeHandle  nh;              //creates our node handle on the arduino --> I think this is similar to the ROS Master for the python code

std_msgs::String message;
ottershaw_masta::Servo debug;
ros::Publisher testMessage("messageRecieved", &message);
ros::Publisher servomessage("legs", &debug);

void setServoDestinations(const ottershaw_masta::Servo& servoInfo)
{
  //digitalWrite(8, HIGH);
  desiredAngle[servoInfo.ID] = servoInfo.angle;
  stepSize[servoInfo.ID] = servoInfo.stepSize;
}

ros::Subscriber<ottershaw_masta::Servo> servoSubscriber("ServoMove", &setServoDestinations);

void setup()
{
  //pinMode(8, OUTPUT);

  //Leg 1
  servos[0].attach(3);
  servos[1].attach(4);
  servos[2].attach(5);

  //Leg 2
  servos[3].attach(9);
  servos[4].attach(10);
  servos[5].attach(13);

  //Leg 3
  servos[6].attach(14);
  servos[7].attach(15);
  servos[8].attach(16);

  //Leg 4
  servos[9].attach(17);
  servos[10].attach(18);
  servos[11].attach(19);

  //Leg 5
  servos[12].attach(20);
  servos[13].attach(21);
  servos[14].attach(22);

  //Leg 6
  servos[15].attach(23);
  servos[16].attach(24);
  servos[17].attach(25);

  //Leg 7
  servos[18].attach(26);
  servos[19].attach(27);
  servos[20].attach(28);

  //Leg 8
  servos[21].attach(29);
  servos[22].attach(30);
  servos[23].attach(31);


  for (int i = 0; i < arraySize; i++)
  {
    currentAngle[i] = 90;
    desiredAngle[i] = 90;
  }    
  
  //Arduino collision prevention offset
  currentAngle[3] = 100;
  desiredAngle[3] = 100;
  currentAngle[6] = 80;
  desiredAngle[6] = 80;
  
  for (int i = 0; i < arraySize; i++)
  {
     servos[i].write(currentAngle[i]);  
  }
  

  nh.initNode();        //creates a node that is the arduino
  nh.subscribe(servoSubscriber);
  nh.advertise(testMessage);
  nh.advertise(servomessage);

  message.data = "";
}

void loop()
{ 
  //servos[5].write(desiredAngle[5]);
  
  for (int i = 0; i < arraySize; i++)
  {
    if (currentAngle[i] < desiredAngle[i])
    {
      currentAngle[i] += stepSize[i];
      servos[i].write(currentAngle[i]);
    }
    else if(currentAngle[i] > desiredAngle[i])
    {
      currentAngle[i] -= stepSize[i];
      servos[i].write(currentAngle[i]);
    }

    //servos[i].write(desiredAngle[i]);
    
  /*  debug.ID=i;
    debug.stepSize=stepSize[i];
    debug.angle=currentAngle[i];
    servomessage.publish(&debug);
    
    nh.spinOnce();
    */
    delay(0);
  }
  
  testMessage.publish(&message);  
  
  //message.data = "yo";

  delay(10);
  nh.spinOnce();

}

