/*
Helper functions for the ARDUINO_ROS_Core module
*/


//Parse comma separated string into string array
//Input: comma separated string, how many comma separated values it contains
//Return: string pointer (array of values) with commas separated out
String* parsedArray = {0};
String* ParseCSV(String toDelimit, int dataSize)
{
  if(dataSize > 0)
  {
    delete [] parsedArray;
  }
  
  //Dynamic allocation on very limited memory -- **be careful here**
  parsedArray = new String[dataSize];
  
  //Parse out CSV
  String parsingString = "";
  int dataCounter = 0;    //How many data points out of total data size we have processed
  
  //Loop through CSV string picking out total of 'dataSize' points and adding to 'parsedArray'
  for(int i = 0; i< toDelimit.length()+1 && dataCounter <= dataSize; i++)
  {
    boolean endOfStringFlag = false;  
    
    //Check if we  have reached a comma, or the end of the string
    if(toDelimit[i] != ',')
    {
      //Add the character we found to the end of the parsed string
      parsingString.concat(toDelimit[i]);
      
      //HACK Special case for last character
      if(i == toDelimit.length() - 1) 
      {
        endOfStringFlag = true;
      }
    }
    
    else
    {
      //HACK -- gets around missing the last character (see special case above in comma check)
      endOfStringFlag = true;
    }
    
    if(endOfStringFlag)
    {
      //PublishDebugMessage(parsingString);
      parsedArray[dataCounter] = parsingString;
      dataCounter++;    //Go onto next data point
      parsingString = "";
      endOfStringFlag = false;
    }
  }

  return parsedArray;

} 

void BlinkLEDs(int interval)
{
  if(currentMillis - previousMillis > interval) 
  {
    // save the last time you blinked the LED 
    previousMillis = currentMillis;   

    // if the LED is off turn it on and vice-versa:
    if (ledState == LOW)
      ledState = HIGH;
    else
      ledState = LOW;

    // set the LED with the ledState of the variable:
    digitalWrite(leftEyePin, ledState);
    digitalWrite(rightEyePin, ledState);
  }
}
