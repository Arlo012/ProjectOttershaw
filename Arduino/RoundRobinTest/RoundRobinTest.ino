#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

ros::NodeHandle nh;

//Publisher
std_msgs::String str_msg;    //TODO can we keep this same variable for all published messages?
ros::Publisher sonar("sonar", &str_msg);
ros::Publisher gyro("gyro", &str_msg);

//Subscriber
void messageCb(const std_msgs::Empty& toggle_msg)
{
  digitalWrite(13, HIGH-digitalRead(13));   // blink the led
}

ros::Subscriber<std_msgs::Empty> sub("toggle_led", &messageCb );
//////////////////////

//Round robin cycler
#define cycleSonar 100U
#define cycleGyro 275U

unsigned long cycleSonarLastMillis = 0;
unsigned long cycleGyroLastMillis = 0;
//////////////////////

boolean cycleCheck(unsigned long *lastMillis, unsigned int cycle) 
{
 unsigned long currentMillis = millis();
 if(currentMillis - *lastMillis >= cycle)
 {
   *lastMillis = currentMillis;
   return true;
 }
 else
   return false;
}

void setup()
{
  //Test LED
  pinMode(13, OUTPUT);
  
  nh.initNode();
  nh.advertise(sonar);
  nh.advertise(gyro);
  
  nh.subscribe(sub);
}

void loop()
{
 if(cycleCheck(&cycleSonarLastMillis, cycleSonar))
 {
  //Get sonar reading here
  char sonarValue[5] = "1000";    //Must return this as a character, or convert it here
  str_msg.data = sonarValue;
  sonar.publish( &str_msg );
 }
 
 if(cycleCheck(&cycleGyroLastMillis, cycleGyro))
 {
  //Get gyro reading here
  char gyroValue[9] = "10,1,5,0";    //TODO decimals are going to be a problem here
  str_msg.data = gyroValue;
  gyro.publish( &str_msg );
 }
  nh.spinOnce();
  delay(10);
}
