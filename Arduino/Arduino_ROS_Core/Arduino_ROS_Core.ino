#include <ros.h>
#include <std_msgs/String.h>

//Begin main class declarat
ros::NodeHandle nh;

//Publisher
std_msgs::String str_msg;    //TODO can we keep this same variable for all published messages?
ros::Publisher sonar("sonar", &str_msg);
ros::Publisher primaryGyro("gyro", &str_msg);    //Warning: 'gyro' has namespace collision with I2C.ino
ros::Publisher debug("ArduinoDebug", &str_msg);
ros::Publisher analog("analog", &str_msg);     //keep publisher for analog instead of step detection? 
   //only one analog type of analog sensor for now.
   
//TODO
//ros::Subscriber<std_msgs::String> sub("servo", &ServoRespond );

//'Round robin' cycler (for polling sensors)
//Use these values to cycle through sensors
#define cycleSonar 100U   //TODO what is appropriate frequency for this check
#define cycleGyro 275U    //TODO what is appropriate frequency for this check
#define cycleAnalog 100U  //TODO more analog inputs per sensor

unsigned long cycleSonarLastMillis = 0;
unsigned long cycleGyroLastMillis = 0;
unsigned long cycleAnalogLastMillis = 0;  //TODO more analog inputs per sensor

//String values returned from read functions
String gyroVal;
String analogVal;
char sensorBuffer[50];      //Holds string to chararray conversion

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

void setup()
{  
  SetupAllSensors();    //See Sensor_Interface.ino
  
  //Put this at the end to avoid sync loss w/ ROScore
  nh.initNode();
  nh.advertise(sonar);
  nh.advertise(primaryGyro);
  nh.advertise(debug);
  nh.advertise(analog);
  //nh.subscribe(sub);
}

void loop()
{
  //External loops
  GyroLoop();      //See MinIMU9AHRS
  
  //Begin round robin sensor polling
  if(cycleCheck(&cycleSonarLastMillis, cycleSonar))
  {
   //str_msg.data = getSonicCharValue();
   sonar.publish( &str_msg );
  }
  
  //Gyro
  if(cycleCheck(&cycleGyroLastMillis, cycleGyro))
  {
   gyroVal = readGyroValues();                 //See Sensor_Interface.ino
   gyroVal.toCharArray(sensorBuffer, 50);      //Converts string to character array in sensorBuffer
   str_msg.data = sensorBuffer;
   primaryGyro.publish( &str_msg );
  }
  
  //Analog Sensors
  if(cycleCheck(&cycleAnalogLastMillis, cycleAnalog))
  {
   analogVal = readGyroValues();                 //See Sensor_Interface.ino
   analogVal.toCharArray(sensorBuffer, 50);      //Converts string to character array in sensorBuffer
   str_msg.data = sensorBuffer;
   primaryGyro.publish( &str_msg );
  }
  
  nh.spinOnce();
  delay(100);
 
}

//Publishes a string over the ArduinoDebug channel
void PublishDebugMessage(String msg)
{
  char buffer [100];                //Long buffer to hold debug message
  msg.toCharArray(buffer, 100);    //Puts string into character buffer
  
  str_msg.data = buffer;         //Places character buffer into str message to send over publisher
  debug.publish(&str_msg );
}


