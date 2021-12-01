#include <Boards.h>
#include <Firmata.h>
#include <LiquidCrystal.h>
#include "CytronMotorDriver.h"
#include <QTRSensors.h>

#define Kp .045 //.03 experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
#define Kd .8    //.8 experiment to determine this, slowly increase the speeds and adjust this value. ( Note: Kp < Kd) 
#define rightMaxSpeed 155 // max speed of the robot
#define leftMaxSpeed 155 // max speed of the robot
#define rightBaseSpeed 70 //60 this is the speed at which the motors should spin when the robot is perfectly on the line
#define leftBaseSpeed 70  //60 this is the speed at which the motors should spin when the robot is perfectly on the line
#define calspeed 75 // max speed of calibration
#define calspeedr 110 // max speed of calibration
#define turnspeed 70 // max speed of calibration
#define ultravalue 25 // distance(inches) ultrasound will start to slow down bot
#define stops 0
#define ENCODER_A 3 
#define ENCODER_B 4

//LCD DISPLAY rs = 53, en = 51, d4 = 49, d5 = 47, d6 = 45, d7 = 43;
LiquidCrystal lcd(53, 51, 49, 47, 45, 43);

//ULTRASOUND
int ultra(void);
int ultrasound;
long duration, inches;
const int trigPin = 22; // Trigger Pin of Ultrasonic Sensor
const int echoPin = 24; // Echo Pin of Ultrasonic Sensor
int ultraled = 29; //LED
int i =0;

// Configure the motor driver.
CytronMD motor1(PWM_DIR, 10, 9);  // PWM 1 = Pin 10, DIR 1 = Pin 9.
CytronMD motor2(PWM_DIR, 12, 11); // PWM 2 = Pin 12, DIR 2 = Pin 11.

//Configure IR Sensor
QTRSensors qtrrc;
const uint8_t NUM_SENSORS = 8;
uint16_t sensorValues[NUM_SENSORS];

//PID Configure
int lastError = 0;
unsigned int sensors[8];
int error;

//ENCODER
volatile signed long rightCount = 0;
double distance;
double turnarounddistance = 3; //CHANGE
double pausedistance = 3; //CHANGE
bool turned = 0;

//BUTTONS
const int buttonpin = 33;
int Aled = 31; //LED
int holding = 0; //FOR PAUSE MAKE 0

void setup()
{

  //ULTRASOUND
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(ultraled, OUTPUT); //LED

  //BUTTON
  pinMode(buttonpin, INPUT_PULLUP); //BUTTON A
  pinMode(Aled, OUTPUT); //LED

  //IR SENSOR
  qtrrc.setTypeRC();
  qtrrc.setSensorPins((const uint8_t[]) {50, 48, 46, 44, 42, 40, 38, 36}, NUM_SENSORS);
  qtrrc.setEmitterPin(52);

  //LCD DISPLAY
  lcd.begin(16, 2);
  lcd.print("hello, world!");

  //ENCODER
  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);
  attachInterrupt(1, rightEncoderEvent, CHANGE);
  
  delay(500);
  Serial.begin(9600);
  Serial.println("STARTING");
 
  calibration();
  button_wait();
   
  } 

void loop()
{
  
  uint16_t position = qtrrc.readLineBlack(sensorValues); // get calibrated readings along with the line position, refer to the QTR Sensors Arduino Library for more details on line position.
  error = position - 3500;  //CHANGED from 2500
  distance = rightCount*.628/950; //METERS

Serial.println(distance);

if(turned ==0){
    if(distance > pausedistance && holding == 0){
      Serial.println("PAUSE LOOP");
      button_wait();
      holding = 1;
    }
    else if(distance > turnarounddistance && holding == 1){
      Serial.println("TURN AROUND LOOP");
      button_wait();
      turn_around();
      distance = 0;
      turned = 1;
      holding = 0; //FOR PAUSE MAKE 0
    }
}
else{
  if(distance > turnarounddistance){
      button_wait();
      turn_around();
      turned = 0;
      holding = 0; //FOR PAUSE MAKE 0
  }
}



Serial.println("MADE IT");
  //PID
  int motorSpeed = Kp * error + Kd * (error - lastError);
  lastError = error;
  int rightMotorSpeed = (rightBaseSpeed + motorSpeed)*ultrasound; //Set Right Motor Speed
  int leftMotorSpeed = (leftBaseSpeed - motorSpeed)*ultrasound; //Set Left Motor Speed

  if (rightMotorSpeed > rightMaxSpeed ) rightMotorSpeed = rightMaxSpeed; // prevent the motor from going beyond max speed
  if (leftMotorSpeed > leftMaxSpeed ) leftMotorSpeed = leftMaxSpeed; // prevent the motor from going beyond max speed
  if (rightMotorSpeed < 0) rightMotorSpeed = 0; // keep the motor speed positive
  if (leftMotorSpeed < 0) leftMotorSpeed = 0; // keep the motor speed positive
  
  if(i==150) {
    ultrasound = ultra();
    Serial.println(distance);
    i=0;
  }
  i++;
  
  motor1.setSpeed(rightMotorSpeed);   // Motor 1
  motor2.setSpeed(leftMotorSpeed);  // Motor 2
  

}

void calibration() {

  int calarray[6][3] = {{calspeedr,0,0}, {-calspeedr,0,6}, {calspeedr,0,3}, {0,calspeed,6}, {0,-calspeed,0}, {0,calspeed,3}}; //{calspeed,0,3},

  for(int j=0; j<=5;j++){
      do{
        qtrrc.read(sensorValues);
        motor1.setSpeed(calarray[j][0]);   // Motor 1
        motor2.setSpeed(calarray[j][1]);  // Motor 2
        qtrrc.calibrate();   
        delay(10);
      } while(sensorValues[calarray[j][2]] < 2200 || (sensorValues[calarray[j][2]]+1) < 2200);
  }
        motor1.setSpeed(0);   // Motor 1
        motor2.setSpeed(0);  // Motor 2
  
delay(2000); // wait for 2s to position the bot before entering the main loop 
       
    Serial.begin(9600);
    Serial.println(" ***Minimum*** ");
    for (int i = 0; i < NUM_SENSORS; i++)
    {
      Serial.print(qtrrc.calibrationOn.minimum[i]);
      Serial.print(' ');
    }
    Serial.println(' ');
    Serial.println(" ***Maximum*** ");
    for (int i = 0; i < NUM_SENSORS; i++)
    {
      Serial.print(qtrrc.calibrationOn.maximum[i]);
      Serial.print(' ');
    }
    Serial.println();
    Serial.println();
    delay(3000);
}

int ultra() {
//USES ULTRASOUND SENSOR TO FIND ANYTHING IN PROXIMITY

  digitalWrite(trigPin, LOW);
  delay(10);
  digitalWrite(trigPin, HIGH);
  delay(10);
  digitalWrite(trigPin, LOW);
 
  // Read the signal from the sensor: a HIGH pulse whose
  // duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(echoPin, INPUT);
  duration = pulseIn(echoPin, HIGH);
 
  // Convert the time into a distance
  //cm = (duration/2) / 29.1;     // Divide by 29.1 or multiply by 0.0343
  inches = (duration/2) / 74;   // Divide by 74 or multiply by 0.0135
     if(inches < ultravalue){
        
        return 0;
        digitalWrite(Aled, HIGH);
     }
     else {
      digitalWrite(Aled, LOW);
      return 1;
     }
}

void rightEncoderEvent() {
//READS ENCODERS TO FIND DISTANCE TRAVELLED

  if (digitalRead(ENCODER_A) == HIGH) {
    if (digitalRead(ENCODER_B) == LOW) {
      rightCount--;
    } else {
      rightCount++;
    }
  } else {
    if (digitalRead(ENCODER_B) == LOW) {
      rightCount++;
    } else {
      rightCount--;
    }
  }
}

void turn_around(){
//TURN MOTORS OFF
//TURN UNTIL LINE IS FOUND

        motor1.setSpeed(0);   // Motor 1
        motor2.setSpeed(0);  // Motor 2 
        delay(500);
  
      do{
        Serial.println(sensorValues[7]);
        motor1.setSpeed(turnspeed);   // Motor 1
        motor2.setSpeed(-turnspeed);  // Motor 2 
        delay(50);
        qtrrc.read(sensorValues);
      } while(sensorValues[7] < 2300);

      rightCount = 0;
  
}

void button_wait() {
//TURN MOTORS OFF
//BUTTON LEDS HIGH
//WAIT FOR BUTTON PRESS

    motor1.setSpeed(0);   // Motor 1
    motor2.setSpeed(0);  // Motor 2         
    digitalWrite(Aled, HIGH);

    int hold = 1;
    Serial.println("BEFORE BUTTON");
    while(hold == 1) {
      if(digitalRead(buttonpin) == LOW){
        delay(100);
        Serial.println("BUTTON");
        if(digitalRead(buttonpin) == LOW){
          hold = 0;
        }
      }
    }

    digitalWrite(Aled, LOW);
    return;
}


  
