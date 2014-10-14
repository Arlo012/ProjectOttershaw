//*****************************************************************************
// Example Sketch for the 0.5" Force Sensitive Resistor
// 3/17/11
// 
// Set up the FSR as R2 of voltage divider circuit with R1 a value of 27k
// Vout connected to Analog input 0
// Prints resistance value of FSR
// 
// WARNING: Resistance value only valid for 27k resistor if using a different
// value, use the equation
// 
//    x = (Rfsr/(R1+Rfsr))*1023
//
// where x is the value from A0, and R1 is the non-FSR resistor, to find the 
// FSR resistance in Ohms. 
//
// To find the actual weight applied to the sensor, refer to the graph on page
// 5 of the datasheet (theoretical values only!) or calibrate the circuit us
// of a known weight. 


//*****************************************************************************

#define FORCE 0

float value = 0;
float resistance = 0;

void setup()
{
  Serial.begin(9600);
  Serial.println("0.5\" Force Sensitive Resistor Test");
}

void loop()
{
  value = analogRead(FORCE);
  resistance = ((26.4*value)/(1-(value/1023.0)));
  Serial.println(resistance,DEC);
  
  delay(200);
}
