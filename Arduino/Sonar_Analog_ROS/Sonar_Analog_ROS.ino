#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
//#include <ottershaw_masta/Analog.h>

//Constants
#define analogPins 4    //How many analog pins (starting at A0) are we going to read?

//LED pins
#define leftEyePin 12
#define rightEyePin 7

//Piezo Buzzer 
#define mouth 6 
//#define mouth2 5

//Begin main class declaration
ros::NodeHandle nh;

//Publisher
std_msgs::String str_msg;
std_msgs::Int32 int_msg;
//ottershaw_masta::Analog analog_msg;

void blink_msg( const std_msgs::String& ArduinoCommand)
{
  digitalWrite(12, HIGH-digitalRead(12));
  digitalWrite(7, HIGH-digitalRead(7));
}

ros::Publisher sonar("sonar", &int_msg);
ros::Publisher debug("ArduinoDebug", &str_msg);
ros::Subscriber<std_msgs::String> sub("ArduinoCommand", &blink_msg);
//ros::Publisher analog("analog", &analog_msg);     //keep publisher for analog instead of step detection? 

//String values returned from read functions
long sonarVal;
int* analogReads;

void setup()
{  
  SetupAllSensors();    //See h_Sensor_Interface.ino
  
  //LED eye setup
  pinMode(leftEyePin, OUTPUT);
  pinMode(rightEyePin, OUTPUT);
  digitalWrite(leftEyePin, HIGH);
  digitalWrite(rightEyePin, HIGH);  
  
  //Piezo Buzzer Mouth
  pinMode(mouth, OUTPUT);
  
  //Put this at the end to avoid sync loss w/ ROScore
  nh.initNode();
  nh.advertise(debug);
  //nh.advertise(analog);
  nh.advertise(sonar);
  nh.subscribe(sub);

  nh.spinOnce();
}

void loop()
{  
  //Sonar
  sonarVal = readSonicScanner();                 //See Sensor_Interface.ino
  int_msg.data = sonarVal;
  sonar.publish(&int_msg);
  nh.spinOnce();
  
  //8-Ohm Speaker tone generator
  toneGradient(sonarVal);
  
  /*
  //Analog Sensors (return one at a time)
  analogReads = readAnalogIns();                 //See Sensor_Interface.ino
  for(int i = 0; i < analogPins; i++) 
  {
    analog_msg.id = i;
    analog_msg.value = analogReads[i];
    analog.publish( &analog_msg );
    nh.spinOnce();
  }
*/
  //delay(100);    //TODO determine if any delay is needed here
}


//Publishes a string over the ArduinoDebug channel
void PublishDebugMessage(String msg)
{
  char buffer [50];                //Long buffer to hold debug message
  msg.toCharArray(buffer, 50);    //Puts string into character buffer
  
  str_msg.data = buffer;         //Places character buffer into str message to send over publisher
  debug.publish(&str_msg );
}


