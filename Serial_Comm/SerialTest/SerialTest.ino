// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  Serial.flush();
  
  int sensorValue = 0;
  for(int i = 0; i < 25; i++) {
    sensorValue = i;
    Serial.println(sensorValue);
    delay(500);        // delay in between reads for stability
  }
}

// the loop routine runs over and over again forever:
void loop() {

}
