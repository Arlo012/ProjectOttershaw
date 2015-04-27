
//Servo loop to test Vibration Piezo sensor. Different scenarios are tested
//to see behavior on live graph done in a Processing sketch.

#include <Servo.h>

//Leg number 1 servo definitions
#define legHip   4
#define legHip2  3
#define legKnee  5

//Leg number 2 servo definitions
#define legHip3   10
#define legHip4   9
#define legKnee2  11

int secondaryHipPos = 90;
int kneePos = 75;

boolean isIncreasing = true;

Servo hipServo;
Servo hipServo2;
Servo hipServo3;
Servo hipServo4;
Servo kneeServo;
Servo knee2Servo;

void setup(){
  Serial.begin(9600);
  
  pinMode(legHip, OUTPUT);
  pinMode(legHip2, OUTPUT);
  pinMode(legHip3, OUTPUT);
  pinMode(legHip4, OUTPUT);
  pinMode(legKnee, OUTPUT);
  pinMode(legKnee2, OUTPUT);
  
  hipServo.attach(legHip);
  hipServo2.attach(legHip2);
  hipServo3.attach(legHip3);
  hipServo4.attach(legHip4);
  kneeServo.attach(legKnee);
  knee2Servo.attach(legKnee2);

  hipServo2.write(secondaryHipPos);
  hipServo4.write(secondaryHipPos);
  kneeServo.write(70);
  knee2Servo.write(kneePos);
}

void loop(){
  //Leg 1 in the air; leg 2 on the ground
  hipServo.write(45);
  //hipServo3.write(90);
  delay(1000);
  //Leg1 on the ground, leg 2 in the air
  hipServo.write(110);
  //hipServo3.write(45);
  delay(1000);
  //Leg1 on the ground, leg 2 on the ground
  //hipServo3.write(90);
  delay(1000);
  //Both legs in the air
  hipServo.write(45);
  //hipServo3.write(45);
  delay(1000);
  //Both legs on the ground
  hipServo.write(110);
  hipServo3.write(90);
  delay(1000);
  //Move Leg 1 up
  hipServo.write(45);
  delay(1000);
  //Leg 1 down
  hipServo.write(110);
  
  
}
