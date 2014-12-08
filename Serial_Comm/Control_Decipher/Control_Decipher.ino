/*possible way to get it working with 115200 baud rate*/
//TODO: research use of coroutines here: http://discuss.littlebits.cc/t/coroutine-library/1170

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
#define DEFAULTBAUDRATE 115200          // Defines The Default Serial Baud Rate (This must match the baud rate specifid in LabVIEW)
#else
#define DEFAULTBAUDRATE 115200
#endif

#include <Servo.h> 
#include "Arduino.h"


//TODO is there an easy way to import this, as we were discussing earlier? 
  //This is gonna get cramped pretty quickly once we add in a couple more sensor classes
/*------------------
Ultrasonic pulse reader class
------------------*/
class SonicScan
{
  private:
    int _trigPin;
    int _echoPin;
    
    //Initialize the sensor by sending a pulse to the trig pin
    //The ultrasonic sensor is triggered by a high pulse > 2 microseconds
    void trigPulse(){
      digitalWrite(_trigPin,LOW); //send low pulse to ensure clean high pulse
      delayMicroseconds(2);
      digitalWrite(_trigPin,HIGH);
      delayMicroseconds(5);
      digitalWrite(_trigPin,LOW);
    }
    
    //Returns distance in centemeters given microsecond time of flight. Assumes air medium
    long msToCm(long microseconds){
      // The speed of sound is 340 m/s or 29 microseconds per centimeter.
      // The ping travels out and back, so to find the distance of the
      // object we take half of the distance travelled.
      return microseconds / 29 / 2;
    }
    
  public:
    String sonicID;
    
    /*
    Parameters:
      sonic_ID: unique string identifying this scanner (TODO: is this necessary?)'
      trigPin: the pin where the trigger pulse must be sent to. See wiring diagram
      echoPin: the pin where the echo response can be measured. See wiring diagram  
   */
    SonicScan(String sonic_ID, int trigPin, int echoPin){
      pinMode(trigPin, OUTPUT);  //Trig pin initialized as output
      pinMode(echoPin, INPUT);  //Echo pin initialized as input
      sonicID = sonic_ID;  //Assigning the given ID to the class instance
      _trigPin = trigPin;
      _echoPin = echoPin;
    }
    
    //Call to return distance from the ultrasonic sensor as measured by signal flight time
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

//Test sonic scanner
SonicScan sonicScan("sonicScan",3,4);

//Test servo
Servo servo1;    

//TODO need to label which of these writes & pinmodes are for what -- I can't tell what part is the ultrasonic, LED, etc 11/23/14
//TODO can this be declared at the program start? Feels weird to be buried inside here, but I guess it isn't hurting anything...
void setup()
{
  pinMode(2,OUTPUT); //attach pin 2 to vcc
  pinMode(5,OUTPUT);  //attach pin 5 GND
  digitalWrite(2, HIGH);
  servo1.attach(8); 
  Serial.begin(9600);
  //Serial.flush();
  pinMode(12, OUTPUT);
  pinMode(6, OUTPUT);
}

char a;          //TODO: label me! Why is this declared out here but char input inside the main loop?

//TODO is text string needed? I see a text passed into the decipher(String text) function, but is this being read anywhere?
String text;    //Received text from serial line
String param;   //Stores parameter sent in with command (holds numbers from passed serial string)
int p[2];       //Integer parameters (integer converted version of param)

//Move the provided servo the given number of 'degree's
//TODO label me. Why are we using servo_index? Thought we wanted a unique ID for each servo?
void move_servo(int servo_index, int degree)
{
  //TODO what is servo_index?
  if(servo_index == 1)
  {
   servo1.write(degree);
  }
}

//Takes input text and determines what kind of command it is, then responds appropriately.
//For example: if read is passed in, the values in p[] are interpreted as which sensor to poll
void decipher(String text)
{
   if(text=="move")
   {
     //move_servo(p[0], p[1]);
      Serial.println(text);
      Serial.println(p[0], DEC);
      Serial.println(p[1], DEC);
      delay(100);
      digitalWrite(12, HIGH);   // turn the LED on (HIGH is the voltage level)
      delay(1000);               // wait for a second
      digitalWrite(12, LOW);    // turn the LED off by making the voltage LOW
      delay(1000);
   }
   
   else if(text == "Read")
   {
      //Serial.println(text);
      //Serial.println(p[0]);
      int ping_read = (int)sonicScan.sonicRead();
      Serial.println(ping_read);
      delay(100);
      digitalWrite(12, HIGH);   // turn the LED on (HIGH is the voltage level)
      delay(1000);               // wait for a second
      digitalWrite(12, LOW);    // turn the LED off by making the voltage LOW
      delay(1000); 
   }
   
   else
   {
     Serial.println("invalid input");
     delay(100);
     digitalWrite(6, HIGH);   // turn the LED on (HIGH is the voltage level)
     delay(1000);             // wait for a second
     digitalWrite(6, LOW);    // turn the LED off by making the voltage LOW
     delay(1000); 
   }
}

//This loop is hard for me to follow -- can you please document what's going on here so we can debug & expand later?
void loop()
{
  while(Serial.available())
  {
    int tag=-1;  //TODO: label tag
    
    //Read serial stream byte, cast to character
    char input = (char)Serial.read();
    
    //Check for stream begin character
    if(input=='#')
    {
      delay(100);
      //TODO we need a better variable name here than 'a'
      a = (char)Serial.read();
      
      //Continue looping through serial stream until end character found
      //TODO what if the end character is lost and we get something like #move!1!50#read!2!!* ?
      while(a != '*')
      {
        if(a != '!')
        {
           if (isDigit(a)) 
           {
          //Serial.println(commandbuffer[index]);
          
          //This character is a digit; append it to the current 
           param.concat(a);
          //Serial.println(param);
           }  
           else
           {
             //Serial.println(a);
             text += a;
             //Serial.println(text);
           }
        }
        
        else
        {
           tag++;
           param = "";
        }
        
        p[tag]=param.toInt();
        a = (char)Serial.read();
      }
        decipher(text);
        text = "";
        //param =""
    }
      
    else 
      {
        text ="";
        while (Serial.available() > 0)
        {
          Serial.read();
        }
        decipher(text);
        //Serial.flush();
        //break;
      }

  }
}

//LIST OF DEBUG SERIAL STRINGS
//#move!2!180*
//#move!8!88*
//#move!8!88*#Read!34*#move!2!180*
//move!2!180*
//#move2180*
//#move!2!180*#move!8!88*
