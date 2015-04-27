/* Knock Sensor
  
   This sketch reads a piezo element to detect a knocking sound. 
   It reads an analog pin and compares the result to a set threshold. 
   If the result is greater than the threshold, it writes
   "knock" to the serial port, and toggles the LED on pin 13.
  
   The circuit:
	* + connection of the piezo attached to analog in 0
	* - connection of the piezo attached to ground
	* 1-megohm resistor attached from analog in 0 to ground

   http://www.arduino.cc/en/Tutorial/Knock
   
   created 25 Mar 2007
   by David Cuartielles <http://www.0j0.org>
   modified 30 Aug 2011
   by Tom Igoe
   
   This example code is in the public domain.

 */
//#include <Servo.h> 


// these constants won't change:
const int ledPin = 13;      // led connected to digital pin 13
const int ledPin2 = 12;
const int knockSensors[] = {A0, A1, A2, A3, A4, A5}; // the piezo is connected to analog pin 0
const int threshold = 15;  // threshold value to decide when the detected sound is a knock or not
const int threshold2 = 200; 
 
//Servo myservo;  // create servo object to control a servo 
                // a maximum of eight servo objects can be created 
 
int pos = 0;    // variable to store the servo position 
boolean isIncreasing = true;
long previousMillis = 0;
long interval = 1;

// these variables will change:
String sensorReadings = "";      // variable to store the value read from the sensor pin
int ledState = LOW;         // variable used to store the last LED status, to toggle the light
int ledState2 = LOW;

void setup() {
 pinMode(ledPin, OUTPUT); // declare the ledPin as as OUTPUT
 Serial.begin(115200);       // use the serial port
 //myservo.attach(3);
}

void loop() {
  
  unsigned long currentMillis = millis();
  
  if(currentMillis - previousMillis > interval){
    // read the sensor and store it in the variable sensorReading:
    for(int i = 0; i<6; i++){
      sensorReadings = String(analogRead(knockSensors[i])) + ",";
    }
    Serial.println(sensorReadings);
    sensorReadings = "";
  }
  //Serial.println(sensorReading2);
  // if the sensor reading is greater than the threshold:
//  if (sensorReading <= threshold) {
//    // toggle the status of the ledPin:
//    //ledState = !ledState;   
//    // update the LED pin itself:        
//    //digitalWrite(ledPin, ledState);
//    // send the string "Knock!" back to the computer, followed by newline
//    Serial.println("standing!");         
//  }
//  else if (sensorReading >= threshold) {
//    // toggle the status of the ledPin:
//    ledState = !ledState;   
//    // update the LED pin itself:        
//    digitalWrite(ledPin, ledState);
//    // send the string "Knock!" back to the computer, followed by newline
//    Serial.println("Knock!");         
//  }  
  //Serial.println("-----------------------");
//  Serial.println("Reading on Leg 2");
//  sensorReading = analogRead(knockSensor2);    
//  Serial.println(sensorReading2);
//  // if the sensor reading is greater than the threshold:
//  if (sensorReading2 <= threshold) {
//    // toggle the status of the ledPin:
//    //ledState = !ledState;   
//    // update the LED pin itself:        
//    //digitalWrite(ledPin, ledState);
//    // send the string "Knock!" back to the computer, followed by newline
//    Serial.println("standing!");         
//  }
//  else if (sensorReading2 >= threshold) {
//    // toggle the status of the ledPin:
//    ledState2 = !ledState2;   
//    // update the LED pin itself:        
//    digitalWrite(ledPin2, ledState2);
//    // send the string "Knock!" back to the computer, followed by newline
//    Serial.println("Knock!");         
//  }  
  
  
  //pos = 0;
  //myservo.write(pos);
  //delay(2000);
  //pos = 90;
  //myservo.write(pos);
//  if(pos == 90)
//  {
//    isIncreasing = false;
//  }
//  else if(pos == 0)
//  {
//    isIncreasing = true;
//  }
//  
//  if(isIncreasing)
//  {
//    myservo.write(pos);
//    pos += 5;
//  }
//  else
//  {
//    myservo.write(pos);
//    pos -= 5;
//  }
}

