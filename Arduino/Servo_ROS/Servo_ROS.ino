#include <ros.h>                  //includes the necessary ROS packages
#include <Servo.h>
#include <ottershaw_masta/Servo.h>

//Servo and leg setup
const int arraySize=24;
Servo servos[arraySize];
int desiredAngle[arraySize];
int currentAngle[arraySize];
int stepSize[arraySize];
ros::NodeHandle  nh;              //creates our node handle on the arduino --> I think this is similar to the ROS Master for the python code

//Servo ROS messages
ottershaw_masta::Servo debug;

//Callback for servo angles
void setServoDestinations(const ottershaw_masta::Servo& servoInfo)
{
  digitalWrite(8, HIGH);
  if(servoAngleSafe(servoInfo.ID, servoInfo.angle))
  {
    desiredAngle[servoInfo.ID] = servoInfo.angle;
    stepSize[servoInfo.ID] = servoInfo.stepSize;
  }
}

//Subscriber (to above callback)
ros::Subscriber<ottershaw_masta::Servo> servoSubscriber("ServoMove", &setServoDestinations);

boolean servoAngleSafe(int ID, int angle)
{
  //DEPRECATED
  return true;
}


void setup()
{
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
  
  //Arduino collision prevention offset on boot
  currentAngle[21] = 85;
  desiredAngle[21] = 85;
  
  for (int i = 0; i < arraySize; i++)
  {
     servos[i].write(currentAngle[i]);  
  }
  
  delay(1000);
  
  nh.initNode();        //creates a node that is the arduino
  nh.subscribe(servoSubscriber);
  
}

void loop()
{ 
  //Servo handler
  for (int i = 0; i < arraySize; i++)
  {
    if (currentAngle[i] < desiredAngle[i])
    {
      currentAngle[i] += stepSize[i];
      servos[i].write(currentAngle[i]);
      if( currentAngle[i] > desiredAngle[i]  )    //Prevent oscillation
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

    delay(0);
  }

  delay(10);
  nh.spinOnce();
}

