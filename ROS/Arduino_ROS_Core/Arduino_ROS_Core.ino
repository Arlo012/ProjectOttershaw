#include <ros.h>
#include <std_msgs/String.h>
  
ros::NodeHandle nh;

//Publisher
std_msgs::String str_msg;    //TODO can we keep this same variable for all published messages?
ros::Publisher sonar("sonar", &str_msg);
ros::Publisher gyro("gyro", &str_msg);
ros::Publisher debug("ArduinoDebug", &str_msg);
//TODO 'step' publisher for step detection
////////////////////////////

//Subscriber (servo movement)
void ServoRespond(const std_msgs::String& servoMsg)
{
  //Parse servo message here, maybe just CSV in form of servo,direction
  String servoMsgString = servoMsg.data;    //Need to access the data within the servo message to modify
  String* csvParsedMessage = ParseCSV(servoMsgString, 2);    //2 values to separate: servo#, degreee
  int servoToMove = csvParsedMessage[0].toInt();
  int angleToMove = csvParsedMessage[1].toInt();
  
  //Lookup servo based on above
  //ServoN.move(angleToMove)
}

ros::Subscriber<std_msgs::String> sub("servo", &ServoRespond );
/////////////////////////


//'Round robin' cycler (for polling sensors)
//Use these values to cycle through sensors
#define cycleSonar 100U   //TODO what is appropriate frequency for this check
#define cycleGyro 275U    //TODO what is appropriate frequency for this check
//Step polling rate here

unsigned long cycleSonarLastMillis = 0;
unsigned long cycleGyroLastMillis = 0;
//Step polling counter here


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

//Ultrasonic declarations here

//Servo constants here

//Gyro code here

void setup()
{ 
  nh.initNode();
  nh.advertise(sonar);
  nh.advertise(gyro);
  nh.advertise(debug);
  
  nh.subscribe(sub);
}

void loop()
{
  //Begin round robin sensor polling
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


//Parse comma separated string into string array
//Input: comma separated string, how many comma separated values it contains
//Return: string pointer (array of values) with commas separated out
String* parsedArray = {};
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
  debug.publish(&str_msg );
}

//TODO Ultrasonic code here

//TODO servo code (if any) here

//TODO gyro code here
