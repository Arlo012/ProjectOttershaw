//***********************************************
//AnalogSensor Class
//Generic class for all analog sensors used
//for Project Ottershaw
//date: 11/13/2014
//***********************************************

#include "Arduino.h"

enum AnalogType
{
  pressure,
  unknown
};
    
class AnalogSensors
{
  private:
    int _analogPin;
    
  public:
    AnalogType _sensorType = unknown; //default sensor type
    
    AnalogSensors(AnalogType sensorType, int analogPin)
    {
      //Construct for the AnalogSensors Class
      pinMode(analogPin, INPUT);  //data pin for analog sensor
      _analogPin = analogPin;  //Readable pin and sensor type
      _sensorType = sensorType;//for python side of code
    }
    
    float sensorRead()
    {
      if(_sensorType==pressure)
      {
        float value = analogRead(_analogPin);
        float resistance = ((26.4 * value)/(1-(value/1023.0)));
        Serial.println(resistance,DEC);
        return resistance;
      }
      else
      {
        Serial.println("-1");
        return -1;
      }
    }    
};

AnalogSensors pressure1(pressure,0);
AnalogSensors unknown1(unknown,1);

void setup()
{
  Serial.begin(9600);
}

void loop()
{
  Serial.println(String(pressure1._sensorType));
  pressure1.sensorRead();
  Serial.println(String(unknown1._sensorType));
  unknown1.sensorRead();
  delay(200);
}
