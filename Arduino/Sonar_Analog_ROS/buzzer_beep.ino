
void beep(unsigned char delayms){
  analogWrite(mouth, 250);      // Almost any value can be used except 0 and 255
                           // experiment to get the best tone
  analogWrite(mouth2, 100);
  delay(delayms);          // wait for a delayms ms
  analogWrite(mouth, 0);       // 0 turns it off
  analogWrite(mouth2, 0);
  delay(delayms);          // wait for a delayms ms   
}  

