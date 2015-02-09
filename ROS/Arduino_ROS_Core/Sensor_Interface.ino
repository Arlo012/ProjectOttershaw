//Libraries & definitions

  #include <SonicScan.h>
  #include <Wire.h>
  #include <LSM303.h>
  #include <L3G.h>
  
//Initialization code

L3G gyro1; //Gyro declarations here


  void Initialize_Sensors()
  {
    //Ultrasonic declarations here
    SonicScan sonicScan("sonicScan",9,10);    //Initialize the ultrasonic scanner to echo/trigger pin

    //Analog declarations here
    //No need to instantiate analog sensors
  }

//Setup code - Function called in the Setup loop of the .ino file

  void Ultrasonic_Setup(int vccPin, int gndPin)
  {
    //Ultrasonic sensor setup
    pinMode(vccPin,OUTPUT); //attach pin to vcc
    pinMode(gndPin,OUTPUT);  //attach pin GND
    digitalWrite(vccPin, HIGH);  //VCC on pin
  }
  
  void Gyro_Setup()
  {
    //Gyro sensor setup
    //Serial.begin(9600);<--- Uncomment for debug purposes only. Only one serial line at a time; thus, it conflicts with ROS serial line
    
    Wire.begin();
  
    if (!gyro1.init())
    {
      //Serial.println("Failed to autodetect gyro type!"); <-- Uncomment for debug only
      while (1);  //Stops code from running if gyro is not properly initialized
    }
  
    gyro1.enableDefault();
  }
  
  //Interfacing the analog pins on the arduino board does not require any special setup

//Data collection and processing functions  

  char[] readGyroValues()
  {
    //Get gyro reading here
    //Concatenate all 3 gyro axial values
    //in one CSV string
    
    String gyroCsvValues = "";  //String to be returned after convertion to a char array.
    
    //Three axial values: x, y , z
    char gyroXVal[5] = {};  
    char gyroYVal[5] = {};
    char gyroZVal[5] = {};
    
    //Convert values to char arrays
    itoa((int)gyro1.g.x, gyroXVal,10);
    itoa((int)gyro1.g.y, gyroYVal,10);
    itoa((int)gyro1.g.z, gyroZVal,10);

    char gyroValue[25] = {};  //TODO Change to publish angles instead of raw data
    for(int i=0; i<5;i++)
    {
      //Write all three gyro values within the same for loop
      //by allocating space in the char array
      //gyroValue[i] = gyroXVal[i];
      //gyroValue[i+5] = gyroYVal[i+5];
      //gyroValue[i+10] = gyroZVal[i+10];
    }
  }
  
  char[] readAnalogIns()
  {
    //Reads all the analog inputs to check for values, 
    //concatenates resistances comma-separated,
    //and ignores inputs without sensors
  
    float rawValue = 0;
    float resistanceVal = 0;
    String analogResistances = "";
  
    //Collect all analog sensor values
    //and place them in a string comma separated
    for(int i = 0; i < 7; i++)
    {
      rawValue = analogRead('A' + (char)i);
      if(rawValue < 0 && rawValue > 32767)
      {
        resistanceVal = ((26.4*rawValue)/(1-(rawValue/1023.0))); //convert analog read to resistance
      
        //TODO you can't convert float to string like this
        //analogResistances += resistanceVal + ","; //concatenate comma-separated resistances
      
      }
      delay(50);     //Keep an eye out for this delay. Might give speed or reading problems later
    }
    
    //remove extra comma at the end of string and add an end-of-string character for easier processing
    analogResistances = analogResistances.substring(0, analogResistances.length() - 1); 
    analogResistances += "!";
  
  
    return &analogResistances;    //Return reference
  }
  
