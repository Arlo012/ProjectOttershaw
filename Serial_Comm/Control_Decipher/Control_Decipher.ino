void setup()
{
 Serial.begin(115200);
 Serial.flush();
}
  char a;
  String text;
  //String param; 
  int p[2];
  void decipher(String text)
  {
     if(text=="move")
     {
        Serial.print("m ");
        Serial.print(p[0]);
        Serial.print(p[1]);
     }
     else if(text=="Read")
     {
        Serial.print('s');
        Serial.print(p[0]);
     }
     else
     { 
       Serial.println("invalid input");
     }
  }
  
//#move!2!180*
//#move!8!88*
//#move!8!88*#Read!34*#move!2!180*
//move!2!180*
//#move2180*
//#move!2!180*#move!8!88*
  
void loop()
{
  while(Serial.available())
  {
     String param;
     int tag=-1;
    if(Serial.read()=='#')
    {
       delay(100);
      a=Serial.read();
      
      while(a != '*')
      {
        if(a != '!')
        {
               if (isDigit(a)) 
               {
              //Serial.println(commandbuffer[index]);
               param.concat(a);
              //Serial.println(param);
               }  
             else
             {
             //Serial.println(a);
               text += a;
               //Serial.println(text);
             }
        }
        else
     {
       tag++;
       param = "";
     }
        p[tag]=param.toInt();
        a=Serial.read();
      }
      decipher(text);
      text ="";
      //param =""
    }
  }
}
  

