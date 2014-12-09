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
  servo1.attach(8); //associate pin 8 with the control pin for one servo 
  Serial.begin(9600);
  //Serial.flush();
  pinMode(12, OUTPUT);//test led pins set to output (you can ignore this if you want)
  pinMode(6, OUTPUT);//test led pins set to output (you can ignore this if you want)
}

  char a;
  char serial_character_grabber; //varliable that pulls the the data from serial character by character
  String text;    //Received text from serial line
  String param;   //Stores parameter sent in with command (holds numbers from passed serial string)
  int p[2];       //Integer parameters (integer converted version of param)
  int command_parameter[2]
  void move_servo(int servo_index, int degree) //simple test function to test servo response form raspberry pi
  {
    if(servo_index == 1)
    {
     servo1.write(degree);//rotate servo a certain amount of degrees
    }
  }
  
  //Takes input text and determines what kind of command it is, then responds appropriately.
  //For example: if read is passed in, the values in p[] are interpreted as which sensor to poll
  void decipher(String text)
  {
     if(text=="move")            //if the raspberry pi commands a servo to move  
     {
       //move_servo(command_parameter[0], command_parameter[1]); // function call to move servo specified by p[1], 
                                                                 //a certain number of degrees specified my p[2]
       // Serial.println(text);                                                    //[debuging purposes, please ignore]
       // Serial.println(command_parameter[0], DEC);  //return the servo ID        //[debuging purposes, please ignore]
       // Serial.println(command_parameter[1], DEC);  //return the amunt rotated   //[debuging purposes, please ignore]
       // delay(100);                                                              //[debuging purposes, please ignore]
       // digitalWrite(12, HIGH);   // turn the LED on (HIGH is the voltage level) //[debuging purposes, please ignore]
       // delay(1000);               // wait for a second                          //[debuging purposes, please ignore]
       // digitalWrite(12, LOW);    // turn the LED off by making the voltage LOW  //[debuging purposes, please ignore]
       //delay(1000);                                                              //[debuging purposes, please ignore]
     }
     
     else if(text == "Read")                                   // if the raspbery pi requests a sensor value
     {
        //this section of code will be in use when there are multiple sensors implemented
        /*
            switch (p[1])                                      // check the id of the sensor that we wish to reas a value from
            {
              //Serial.println(text);                          //[debuging purposes, please ignore]
                case 1:                                        //for example if we want the ultrasonlic sensor to return a 
                                                               //measured distance to the raspbaerry pi p[1] will hold the id for it
                  int ping_read = (int)sonicScan.sonicRead();  //call sensor ID 1 function
                  Serial.println(ping_read);                   //return its value
                break;
                
                case 2:
                  //call sensor ID 2 function
                  //return its value
                break;
     
                case 3:
                  //call sensor ID 3 function
                  //return its value 
                break;
                 
              default: 
              Serial.println("No Such Sensor");     //default error message to raspbery pi if no such sensor of the specified id 
                                                    //exists
            }
        */
        Serial.println(command_parameter[0]);       // unique ID that the rspbery pi generated, the arduino will send back
        int ping_read = (int)sonicScan.sonicRead(); // read the sensor
        Serial.println(ping_read);                  // return maseured value
     }
     
     else
     { 
       Serial.println("invalid input"); //if there was an issue with the command (such as currupted data stream, or call to a non existant sensor/servo)
       delay(100);
       digitalWrite(6, HIGH);                     // turn the LED on (HIGH is the voltage level)
       delay(1000);                               // wait for a second
       digitalWrite(6, LOW);                      // turn the LED off by making the voltage LOW
       delay(1000); 
     }
  }
/* sample inputs 
#move!2!180*
#move!8!88*
#move!8!88*#Read!34*#move!2!180*
move!2!180*
#move2180*
#move!2!180*#move!8!88*
  */
void loop()
{
  while(Serial.available())
  {
    //String param;
    int tag=-1;
    char input = (char)Serial.read();  //grabs first character from serial
    
    if(input=='#')                     //checks if its the start of a new command
    {
      delay(100);
      a = (char)Serial.read();         // grab next character fraom serial
      
      while(serial_character_grabber != '*')                  // as long as we are not signaled that it is the end of a command
      {
        if(serial_character_grabber != '!')                   //if it is not the end of a parameter 
        {
           if (isDigit(serial_character_grabber))             //if it is a digit (signifying, a sensor id, or degree amount that 
                                       //a servo should rotate)
           {
          //Serial.println(commandbuffer[index]);   //[debuging purposes, please ignore]
           param.concat(serial_character_grabber);                         //conacatonate the value to a "param" variable
          //Serial.println(param);                  //[debuging purposes, please ignore]
           }  
           else                                     //else if what we've pulled from serial is not a digit 
                                                    //(or implicitly a '!', '#', or '*')
           {
             //Serial.println(a);                   //[debuging purposes, please ignore]
             text += serial_character_grabber;      //the character pulled from serial must be text possibly specifying a "read" or "move" command 
                                                    //so append the character to the other characters in the "text" variable
             //Serial.println(text);                //[debuging purposes, please ignore]
           }
        }
        
        else                                         //if a separating '!' cahracter was found signifying another parameter 
                                                     //(most likely an integer for servo movement or sensor identification ) 
        {
           command_parameter[tag]=param.toInt();     //convert the parameter (String) to and interger
           tag++;                                    // increment tag index for the array holding the parameters
           param = "";                               // empty the parameter string to make room for the next incomming parameter
        }
        //command_parameter[tag]=param.toInt();                      //convert the parameter (String) to and interger
        serial_character_grabber = (char)Serial.read();              // readin the next value from serial
      }
                                                      //when the end command character has been detected 
        decipher(text);                               // call the decipher function
        text = "";                                    //empty out the "text" 
        //param =""
    }
      
    else                                             // else if the starting character is NOT '#' which means 
                                                     //that there was some error in the command sent from the raspberry pi
      {
        text ="";                                   //empty out the "text variable" signifying an error in the decipher function 
        while (Serial.available() > 0)              //if there are still values in the serial buffer
        {
          Serial.read();                            //clear the serial buffer of all queued commands
        }
        decipher(text);                             // call the decipher function ehich will output an error message
        //Serial.flush();                           //[debuging purposes, please ignore]
        //break;                                    //[debuging purposes, please ignore]
      }
  }
}
