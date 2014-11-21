/*possible way to get it working with 115200 baud rate*/

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
#define DEFAULTBAUDRATE 115200          // Defines The Default Serial Baud Rate (This must match the baud rate specifid in LabVIEW)
#else
#define DEFAULTBAUDRATE 115200
#endif

#include <Servo.h> 
#include "Arduino.h"

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

SonicScan sonicScan("sonicScan",3,4);
Servo servo1;

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

  char a;
  String text;    //Received text from serial line
  String param;   //Stores parameter sent in with command (holds numbers from passed serial string)
  int p[2];       //Integer parameters (integer converted version of param)
  
  void move_servo(int servo_index, int degree)
  {
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
       delay(1000);               // wait for a second
       digitalWrite(6, LOW);    // turn the LED off by making the voltage LOW
       delay(1000); 
     }
  }
  
//#move!2!180*
//#move!8!88*
//#move!8!88*#Read!34*#move!2!180*
//move!2!180*
//#move2180*
//#move!2!180*#move!8!88*
  
void loop()
{
  while(Serial.available())
  {
    //String param;
    int tag=-1;
    char input = (char)Serial.read();
    
    if(input=='#')
    {
      delay(100);
      a = (char)Serial.read();
      
      while(a != '*')
      {
        if(a != '!')
        {
           if (isDigit(a)) 
           {
          //Serial.println(commandbuffer[index]);
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
