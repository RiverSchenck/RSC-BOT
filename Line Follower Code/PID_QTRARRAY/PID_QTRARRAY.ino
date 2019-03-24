#include <Boards.h>
#include <Firmata.h>

#include <QTRSensors.h>

#define Kp .025 // experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
#define Kd 8    // experiment to determine this, slowly increase the speeds and adjust this value. ( Note: Kp < Kd) 
#define rightMaxSpeed 150 // max speed of the robot
#define leftMaxSpeed 150 // max speed of the robot
#define rightBaseSpeed 100 // this is the speed at which the motors should spin when the robot is perfectly on the line
#define leftBaseSpeed 100  // this is the speed at which the motors should spin when the robot is perfectly on the line


//L293 Connection   
  const int rightmotor1       = 6;  
  const int rightmotor2       = 5; 
  const int rightmotorPWM     = 7;
  const int leftmotor1        = 9; 
  const int leftmotor2        = 8; 
  const int leftmotorPWM      = 10;


QTRSensors qtrrc;
const uint8_t NUM_SENSORS = 8;
uint16_t sensorValues[NUM_SENSORS];

void setup()
{
  qtrrc.setTypeRC();
  qtrrc.setSensorPins((const uint8_t[]) {50, 48, 46, 44, 42, 40, 38, 36}, NUM_SENSORS);
  qtrrc.setEmitterPin(52);
  delay(500);
  Serial.begin(9600);

  pinMode(rightmotor1, OUTPUT);
  pinMode(rightmotor2, OUTPUT);
  pinMode(leftmotor1, OUTPUT);
  pinMode(leftmotor2, OUTPUT);
  pinMode(rightmotorPWM, OUTPUT);
  pinMode(leftmotorPWM, OUTPUT);
  
  
  int i;
for (int i = 0; i < 100; i++) // calibrate for sometime by sliding the sensors across the line, or you may use auto-calibration instead

  /* comment this part out for automatic calibration 
  if ( i  < 25 || i >= 75 ) // turn to the left and right to expose the sensors to the brightest and darkest readings that may be encountered
     turn_right();  
   else
     turn_left(); */ 
   qtrrc.calibrate();   
   delay(20);
  
delay(2000); // wait for 2s to position the bot before entering the main loop 
    
    /* comment out for serial printing
    
    Serial.begin(9600);
    for (int i = 0; i < NUM_SENSORS; i++)
    {
      Serial.print(qtrrc.calibratedMinimumOn[i]);
      Serial.print(' ');
    }
    Serial.println();

    for (int i = 0; i < NUM_SENSORS; i++)
    {
      Serial.print(qtrrc.calibratedMaximumOn[i]);
      Serial.print(' ');
    }
    Serial.println();
    Serial.println();
    */
  } 

int lastError = 0;

void loop()
{
  unsigned int sensors[6];
  uint16_t position = qtrrc.readLineBlack(sensorValues); // get calibrated readings along with the line position, refer to the QTR Sensors Arduino Library for more details on line position.
  int error = position - 2500;
Serial.println(position);
  int motorSpeed = Kp * error + Kd * (error - lastError);
  lastError = error;

  int rightMotorSpeed = rightBaseSpeed + motorSpeed;
  int leftMotorSpeed = leftBaseSpeed - motorSpeed;
  
    if (rightMotorSpeed > rightMaxSpeed ) rightMotorSpeed = rightMaxSpeed; // prevent the motor from going beyond max speed
  if (leftMotorSpeed > leftMaxSpeed ) leftMotorSpeed = leftMaxSpeed; // prevent the motor from going beyond max speed
  if (rightMotorSpeed < 0) rightMotorSpeed = 0; // keep the motor speed positive
  if (leftMotorSpeed < 0) leftMotorSpeed = 0; // keep the motor speed positive
  
   {
  //digitalWrite(motorPower, HIGH); // move forward with appropriate speeds
  digitalWrite(rightmotor1, LOW);
  digitalWrite(rightmotor2, HIGH);
  analogWrite(rightmotorPWM, leftMotorSpeed);
  //digitalWrite(motorPower, HIGH);
  digitalWrite(leftmotor1, LOW);
  digitalWrite(leftmotor2, HIGH);
  analogWrite(leftmotorPWM, rightMotorSpeed);
}
}
  
