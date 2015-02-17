#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <ottershaw/Gyro.h>
#include <ottershaw/Accel.h>
#include <ottershaw/Analog.h>

//Begin main class declaration
ros::NodeHandle nh;

//Publisher
std_msgs::String str_msg;
std_msgs::Int32 int_msg;
//ottershaw::Gyro gyro_msg;
ottershaw::Accel accel_msg;
ottershaw::Analog analog_msg;

ros::Publisher sonar("sonar", &int_msg);
//ros::Publisher primaryGyro("gyro", &gyro_msg);    //Warning: 'gyro' has namespace collision with I2C.ino
//ros::Publisher accelerometer("accel", &accel_msg);
ros::Publisher debug("ArduinoDebug", &str_msg);
ros::Publisher analog("analog", &analog_msg);     //keep publisher for analog instead of step detection? 

//TODO
//ros::Subscriber<std_msgs::String> sub("servo", &ServoRespond );

//String values returned from read functions
long sonarVal;
int* analogReads;
//char sensorBuffer[50];      //Holds string to chararray conversion

void setup()
{  
  SetupAllSensors();    //See h_Sensor_Interface.ino
  
  //Put this at the end to avoid sync loss w/ ROScore
  nh.initNode();
  //nh.advertise(primaryGyro);
  //nh.advertise(accelerometer);
  nh.advertise(debug);
  nh.advertise(analog);
  nh.advertise(sonar);

  nh.spinOnce();
}

void loop()
{  

  //Sonar
  sonarVal = readSonicScanner();                 //See Sensor_Interface.ino
  int_msg.data = sonarVal;
  sonar.publish(&int_msg);
  nh.spinOnce();
  
  /*
  //Gyro
  readGyroValues();                 //See Sensor_Interface.ino
  primaryGyro.publish( &gyro_msg );
  nh.spinOnce();
  */
  
  //Analog Sensors (return one at a time)
  analogReads = readAnalogIns();                 //See Sensor_Interface.ino
  for(int i = 0; i < sizeof(analogReads); i++) 
  {
    analog_msg.id = i;
    analog_msg.value = analogReads[i];
    analog.publish( &analog_msg );
    nh.spinOnce();
  }
  nh.spinOnce();
  delay(100);
}


//Publishes a string over the ArduinoDebug channel
void PublishDebugMessage(String msg)
{
  char buffer [50];                //Long buffer to hold debug message
  msg.toCharArray(buffer, 50);    //Puts string into character buffer
  
  str_msg.data = buffer;         //Places character buffer into str message to send over publisher
  debug.publish(&str_msg );
}


