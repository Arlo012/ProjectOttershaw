#include <Wire.h>
#include <LSM303.h>
#include <L3G.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <Servo.h> 
#include <SonicScan.h>

//Begin main class declarat
ros::NodeHandle nh;

//Publisher
std_msgs::String str_msg;    //TODO can we keep this same variable for all published messages?
//ros::Publisher sonar("sonar", &str_msg);
//ros::Publisher gyro("gyro", &str_msg);
//ros::Publisher debug("ArduinoDebug", &str_msg);
//ros::Publisher analog("analog", &str_msg);     //keep publisher for analog instead of step detection? 
                                              //only one analog type of analog sensor for now.
//TODO 'step' publisher for step detection -- done
////////////////////////////

  
  Servo servos[24];
void ServoRespond(const std_msgs::String& servoMsg)
{
  //Parse servo message here, maybe just CSV in form of servo,direction
  String servoMsgString = servoMsg.data;    //Need to access the data within the servo message to modify
  String* csvParsedMessage = ParseCSV(servoMsgString, 2);    //2 values to separate: servo#, degreee
  int servoToMove = csvParsedMessage[0].toInt(); //This is the servo that needs to be moved
  int angleToMove = csvParsedMessage[1].toInt(); //This is the angle desired for the servo 

  servos[servoToMove].write(angleToMove);//This puts the desired servo at the right angle

}

ros::Subscriber<std_msgs::String> sub("servo", &ServoRespond );
/////////////////////////


//'Round robin' cycler (for polling sensors)
//Use these values to cycle through sensors
#define cycleSonar 100U   //TODO what is appropriate frequency for this check
#define cycleGyro 275U    //TODO what is appropriate frequency for this check
#define cycleAnalog 100U  //TODO more analog inputs per sensor


unsigned long cycleSonarLastMillis = 0;
unsigned long cycleGyroLastMillis = 0;
unsigned long cycleAnalogLastMillis = 0;  //TODO more analog inputs per sensor

//Checks if it is time to run an event by passing in last time completed, and unsigned refresh period
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
//////////////////////

//Servo constants here
//Servo servo1;


void setup()
{ 
  //nh.initNode();
  //nh.advertise(sonar);
  //nh.advertise(gyro);
  //nh.advertise(debug);
  //nh.advertise(analog);
  
  //nh.subscribe(sub);
  

  
  //Servo setup
 // servo1.attach(5); //associate pin 8 with the control pin for one servo 
 
 // servos[0].attach(0);
 /*
 for (int cntr = 1; cntr<7;cntr++) // 5 should be 24, 5 is for testing son
 {
   servos[cntr].attach(cntr);
         PublishDebugMessage("apple");

 }*/
  
}

void loop()
{
  /*
  //Begin round robin sensor polling
  if(cycleCheck(&cycleSonarLastMillis, cycleSonar))
  {
   //Get sonar reading here
   int sonicInt = (int)sonicScan.sonicRead();    //Must return this as a character, or convert it here
   char sonicCharValue[5] = {};
   itoa(sonicInt,sonicCharValue, 10);    //Convert integer to character array
   //TODO test this itoa
   
   str_msg.data = sonicCharValue;
   sonar.publish( &str_msg );
  }
 
  if(cycleCheck(&cycleGyroLastMillis, cycleGyro))
  {
 
  }
  
  if(cycleCheck(&cycleAnalogLastMillis, cycleAnalog))
  {
    //Get analog reading here
    char buff [50];
    String* analogValues = readAnalogIns();
    analogValues->toCharArray(buff, 50);    //Function converts string value to character array in buff
    str_msg.data = buff;                    //Set buff as output data
    analog.publish( &str_msg );
  }*/
 
  //nh.spinOnce();
  delay(100);
  
  
  
}


//Parse comma separated string into string array
//Input: comma separated string, how many comma separated values it contains
//Return: string pointer (array of values) with commas separated out
String* parsedArray = {0};
String* ParseCSV(String toDelimit, int dataSize)
{
  if(dataSize > 0)
  {
    delete [] parsedArray;
  }
  
  //Dynamic allocation on very limited memory -- **be careful here**
  parsedArray = new String[dataSize];
  
  //Parse out CSV
  String parsingString = "";
  int dataCounter = 0;    //How many data points out of total data size we have processed
  
  //Loop through CSV string picking out total of 'dataSize' points and adding to 'parsedArray'
  for(int i = 0; i< toDelimit.length() && dataCounter <= dataSize; i++)
  {
    boolean endOfStringFlag = false;
    
    //Check if we  have reached a comma, or the end of the string
    if(toDelimit[i] != ',')
    {
      //Add the character we found to the end of the parsed string
      parsingString.concat(toDelimit[i]);
      
      //Special case for last character (hack)
      if(i == toDelimit.length() - 1) 
      {
        endOfStringFlag = true;
      }
    }
    else
    {
      //HACK -- gets around missing the last character (see special case above in comma check)
      endOfStringFlag = true;
    }
    
    if(endOfStringFlag)
    {
      PublishDebugMessage(parsingString);
      
      parsedArray[dataCounter] = parsingString;
      dataCounter++;    //Go onto next data point
      parsingString = "";
      endOfStringFlag = false;
    }
  }
  
  return parsedArray;
}

//Publishes a string over the ArduinoDebug channel
void PublishDebugMessage(String msg)
{
  char buffer [40];                //Long buffer to hold debug message
  msg.toCharArray(buffer, 40);    //Puts string into character buffer
  
  str_msg.data = buffer;         //Places character buffer into str message to send over publisher
  
  //RE-enable me once ROS working
  //debug.publish(&str_msg );
}



