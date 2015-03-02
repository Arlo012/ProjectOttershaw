//Constants for linear tone implementation
#define toneDuration 10
#define freqSlope 3.63 //slope in Hz/cm for gradient
#define freqConst 1007  //constant term of linear equation
/*
void beep(unsigned char delayms)
{
  analogWrite(mouth, 250);      // Almost any value can be used except 0 and 255
                           // experiment to get the best tone
  analogWrite(mouth2, 100);
  delay(delayms);          // wait for a delayms ms
  analogWrite(mouth, 0);       // 0 turns it off
  analogWrite(mouth2, 0);
  delay(delayms);          // wait for a delayms ms   
}  */

void toneGradient(long sonarReading)
{
  //Function that linearly changes the frequency of the tone 
  //inversely proportional to the distance given by the sonicScanner
  //Equation for line: y = 1007 - 3.63*x, which defines freq value
  if((int)sonarReading <= 100)
  {
    int toneFreq = freqConst - (int)(freqSlope*sonarReading);
    tone(mouth, toneFreq, toneDuration);
  }
}
