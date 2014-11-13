// Pin 13 has an LED connected on most Arduino boards.
int ledState = LOW;  
int led = 13;
long previousMillis = 0;
long interval = 1000;  

// the setup routine runs once when you press reset:
void setup() {          
  // initialize the digital pin as an output.
  pinMode(led, OUTPUT);  
  Serial.begin(115200); //This initialices the USB as a serial port
}


// the loop routine runs over and over again forever:
void loop() {
  char incomingByte = (char)Serial.read();
  //Serial.println(incomingByte);
  //delay(250);
  if(incomingByte=='1'){
    //digitalWrite(led,HIGH);
    //delay(5000);
    //digitalWrite(led,LOW);
    Serial.print("1");
  }
 
}
