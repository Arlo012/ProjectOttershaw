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
  digitalWrite(8, HIGH);
  if(servoAngleSafe(servoInfo.ID, servoInfo.angle))
  {
    desiredAngle[servoInfo.ID] = servoInfo.angle;
    stepSize[servoInfo.ID] = servoInfo.stepSize;
  }

}


boolean servoAngleSafe(int ID, int angle)
{/*
  if(ID == 1 || ID == 4 || ID == 7 || ID == 10 || ID == 13 || ID == 16 || ID == 19 || ID == 22)
  {
    if(angle < 10)
    {
      return false;
    }
  }*/
  return true;
}

ros::Subscriber<ottershaw_masta::Servo> servoSubscriber("ServoMove", &setServoDestinations);

void setup()
{
  //pinMode(8, OUTPUT);

  //Leg 1
  servos[0].attach(24);
  servos[1].attach(22);
  servos[2].attach(26);


  //Leg 2
  servos[3].attach(28);
  servos[4].attach(30);
  servos[5].attach(32);

  //Leg 3
  servos[6].attach(36);
  servos[7].attach(34);
  servos[8].attach(38);

  //Leg 4
  servos[9].attach(27);
  servos[10].attach(29);
  servos[11].attach(31);

  //Leg 5
  servos[12].attach(37);  
  servos[13].attach(35);  
  servos[14].attach(33);

  //Leg 6
  servos[15].attach(39);
  servos[16].attach(41);
  servos[17].attach(40);

  //Leg 7
  servos[18].attach(23);
  servos[19].attach(25);
  servos[20].attach(53);

  //Leg 8
  servos[21].attach(49);
  servos[22].attach(51);
  servos[23].attach(47);

  for (int i = 0; i < arraySize; i++)
  {
    if(i % 3 == 1)    //Angle up to prevent crashing into floor 
    {
      currentAngle[i] = 20;
    }
    else if(i % 3 == 2)    //Angle up to prevent crashing into floor 
    {
      currentAngle[i] = 45;
    }
    else
    {
      currentAngle[i] = 90;
      desiredAngle[i] = 90;
    }

  }    
  
  //Arduino collision prevention offset
  currentAngle[21] = 85;
  desiredAngle[21] = 85;
  
  
  for (int i = 0; i < arraySize; i++)
  {
     servos[i].write(currentAngle[i]);  
  }
  
  delay(1000);
  
  //nh.getHardware()->setBaud(9600);
  nh.initNode();        //creates a node that is the arduino
  nh.subscribe(servoSubscriber);
  nh.advertise(testMessage);
  nh.advertise(servomessage);

  message.data = "";
}

void loop()
{ 
  for (int i = 0; i < arraySize; i++)
  {
    if (currentAngle[i] < desiredAngle[i])
    {
      currentAngle[i] += stepSize[i];
      servos[i].write(currentAngle[i]);
      if( currentAngle[i] > desiredAngle[i]  )
      {
        servos[i].write(desiredAngle[i]);
      }
    }
    else if(currentAngle[i] > desiredAngle[i])
    {
      currentAngle[i] -= stepSize[i];
      servos[i].write(currentAngle[i]);
      if( currentAngle[i] < desiredAngle[i]  )
      {
        servos[i].write(desiredAngle[i]);
      }
    }
    
    else
    {
      servos[i].write(currentAngle[i]);
    }
    
    //servos[i].write(desiredAngle[i]);
    
    //debug.ID=i;
    //debug.stepSize=stepSize[i];
    //debug.angle=currentAngle[i];
    //servomessage.publish(&debug);
    
    //nh.spinOnce();

    delay(0);
  }
  //testMessage.publish(&message);  

  //message.data = "yo";

  delay(10);
  nh.spinOnce();
}

