#include <ros.h>
#include <std_msgs/String.h>
#include <Servo.h> 
  
ros::NodeHandle nh;

//Publisher
std_msgs::String str_msg;    //TODO can we keep this same variable for all published messages?
ros::Publisher sonar("sonar", &str_msg);
ros::Publisher gyro("gyro", &str_msg);
ros::Publisher debug("ArduinoDebug", &str_msg);
ros::Publisher analog("analog", &str_msg);     //keep publisher for analog instead of step detection? 
                                              //only one analog type of analog sensor for now.
//TODO 'step' publisher for step detection -- done
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
#define cycleAnalog 100U
//Step polling rate here

unsigned long cycleSonarLastMillis = 0;
unsigned long cycleGyroLastMillis = 0;
unsigned long cycleAnalogLastMillis = 0;  //pointer for analog sensors
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
SonicScan sonicScan("sonicScan",9,10);    //Initialize the ultrasonic scanner to echo/trigger pin

//Servo constants here
Servo servo1;

//Gyro code here

void setup()
{ 
  nh.initNode();
  nh.advertise(sonar);
  nh.advertise(gyro);
  nh.advertise(debug);
  
  nh.subscribe(sub);
  
  //Ultrasonic sensor setup
  pinMode(8,OUTPUT); //attach pin 8 to vcc
  pinMode(11,OUTPUT);  //attach pin 11 GND
  digitalWrite(8, HIGH);  //VCC on pin 8
  
  //Servo setup
  servo1.attach(5); //associate pin 8 with the control pin for one servo 
  Serial.begin(9600);
}

void loop()
{
  //Begin round robin sensor polling
  if(cycleCheck(&cycleSonarLastMillis, cycleSonar))
  {
   //Get sonar reading here
   char sonarValue[5] = (int)sonicScan.sonicRead();    //Must return this as a character, or convert it here
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
  
  if(cycleCheck(&cycleAnalogLastMillis, cycleAnalog))
  {
    //Get analog reading here
    char buff [50];
    String analogValues = readAnalogIns();
    str_msg.data = analogValues.toCharArray(buff, 50);
    analog.publish( &str_msg );
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
class SonicScan
{
  
  private:
    int _trigPin;
    int _echoPin;
    
    void trigPulse(){
      //Initialize the sensor by sending a pulse to the trig pin
      //The ultrasonic sensor is triggered by a high pulse > 2 microseconds
      digitalWrite(_trigPin,LOW); //send low pulse to ensure clean high pulse
      delayMicroseconds(2);
      digitalWrite(_trigPin,HIGH);
      delayMicroseconds(5);
      digitalWrite(_trigPin,LOW);
    }
    
    long msToCm(long microseconds){
      // The speed of sound is 340 m/s or 29 microseconds per centimeter.
      // The ping travels out and back, so to find the distance of the
      // object we take half of the distance travelled.
      return microseconds / 29 / 2;
    }
    
  public:
    String sonicID;
    
    SonicScan(String sonic_ID, int trigPin, int echoPin){
      pinMode(trigPin, OUTPUT);  //Trig pin initialized as output
      pinMode(echoPin, INPUT);  //Echo pin initialized as input
      sonicID = sonic_ID;  //Assigning the given ID to the class instance
      _trigPin = trigPin;
      _echoPin = echoPin;
    }
    
    long sonicRead(){
      trigPulse();
      //Returns distance reading of ultrasonic sensor in centimeters. 
      long duration, cm;
      
      //Duration in time from sending the ping to the reception of the echo off of an object
      duration = pulseIn(_echoPin, HIGH);
  
      //Unit convertion from duration to distance in centimeters
      cm = msToCm(duration);
      
      return cm;
    }
};

//TODO Analog code here
String* readAnalogIns()
{
  //Reads all the analog inputs to check for values, 
  //concatenates resistances comma-separated,
  //and ignores inputs without sensors
  
  float rawValue = 0;
  float resistanceVal = 0;
  String analogResistances = "";
  
  //Collect all analog sensor values
  //and place them in a string comma separated
  for(i=0; i < 7; i++)
  {
    rawValue = analogRead(i);
    if(rawValue < 0 && rawValue > 32767)
    {
      resistanceVal = ((26.4*rawValue)/(1-(rawValue/1023.0))); //convert analog read to resistance
      analogResistances += resistanceVal + ","; //concatenate comma-separated resistances
    }
    delay(50) //Keep an eye out for this delay. Might give speed or reading problems later
  }
  //remove extra comma at the end of string and add an end-of-string character for easier processing
  analogResistances = analogResistances.substring(0, analogResistances.length()-1); 
  analogResistances += "!"
  
  return analogResistances
}

//TODO gyro code here

//TODO servo code (if any) here
//Do we still need this function? or does ROS take care of indexing the servos
void move_servo(int servo_index, int degree) 
{
    if(servo_index == 1)
    {
     servo1.write(degree);//rotate servo a certain amount of degrees
    }
}

