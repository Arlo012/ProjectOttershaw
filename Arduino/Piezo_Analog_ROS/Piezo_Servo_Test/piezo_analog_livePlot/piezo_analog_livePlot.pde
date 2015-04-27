import processing.serial.*;
import java.util.*;
import java.text.*;

PrintWriter output;
String filename;
Serial myPort;
int xPos = 0;
String toWrite;
int numReadings = 0;
static int numSensors = 2;

void setup(){
  size(1500,300);
  println(Serial.list());
  myPort = new Serial(this, Serial.list()[0], 115200);

  myPort.bufferUntil('\n');

  //background(0);
  filename = "/home/jorgee/ProjectOttershaw/Arduino/Piezo_Analog_ROS/Piezo_Servo_Test/piezo_analog_livePlot/Piezo_FourLegMarch_WithSensor_1-5sec";
  output = createWriter(filename + ".csv");


  frameRate(9999999);
}

void draw(){
  output.print(millis() + "," + toWrite + "\n");
}

void keyPressed(){
  output.flush();
  output.close();
  exit();
}


boolean itemToWrite = false;    //Something to write out to file
//Slower than draw()
void serialEvent(Serial myPort){
  String inString = myPort.readStringUntil('\n');
  println(inString);
  if(inString != null){
    inString = trim(inString);
    toWrite = inString;    //Write this in the draw loop
    
    float inByte = float(inString);
    inByte = map(inByte,0,1023,0,height);
    
    stroke(127,34,255);
    line(xPos-1,height,xPos,height-4*inByte-25);
    
    if(xPos>=width){
      xPos = 0;
      background(0);
    }
    else{
      xPos++;
    }
    
  }
}
