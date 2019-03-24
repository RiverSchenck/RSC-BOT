#include <QTRSensors.h>

//SPEEDS MAX 255
  int motorspeed_A;
  int motorspeed_B;
  int straight_speed =  63;      
  int soft_speed =      68;
  int slight_speed =    76;
  int moderate_speed =  84;
  int hard_speed =      92;

  
//QTRRC8 ARRAY
//  #define NUM_SENSORS   8     // number of sensors used
//  #define TIMEOUT       2500  // waits for 2500 microseconds for sensor outputs to go low
//  #define EMITTER_PIN   52     // emitter is controlled by digital pin 2
  #define irvalue       2200


//L293 Connection   
  const int motorA1      = 6;  
  const int motorA2      = 5; 
  const int motorAspeed  = 7;
  const int motorB1      = 9; 
  const int motorB2      = 8; 
  const int motorBspeed  = 10;



  
//===========================================================================================
//HERE WE GO
//===========================================================================================
QTRSensors qtrrc;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];


void setup()
{

    qtrrc.setTypeRC();
    qtrrc.setSensorPins((const uint8_t[]) {50, 48, 46, 44, 42, 40, 38, 36}, SensorCount);
    qtrrc.setEmitterPin(52);
    delay(500);
    
    Serial.begin(9600); // set the data rate in bits per second for serial data transmission
  
    pinMode(motorA1, OUTPUT);
    pinMode(motorA2, OUTPUT);
    pinMode(motorB1, OUTPUT);
    pinMode(motorB2, OUTPUT);
    
    //delay(1000);
}


void loop()
{
      read_array();
          for (unsigned char i = 0; i < SensorCount; i++)
          {
            Serial.print(sensorValues[i]);
            Serial.print('\t'); // tab to format the raw data into columns in the Serial monitor
          }
          Serial.println();
      //delay(250);
      
} //end loop




  void read_array() {
  
     qtrrc.read(sensorValues);
     
    //-------------------------------------------
    //MOTOR A
    //-------------------------------------------
        //HARD RIGHT
        if (sensorValues[0] > irvalue) {
              Serial.print("Hard Right");
              motorspeed_A = hard_speed; 
        }
        //MODERATE RIGHT
        else if (sensorValues[1] > irvalue) {
              Serial.print("Moderate Right");
              motorspeed_A = moderate_speed; 
        }
        //SLIGHT RIGHT
        else if (sensorValues[2] > irvalue) {
              Serial.print("Slight Right");
               motorspeed_A = slight_speed; 
        }
        //SOFT RIGHT
        else if(sensorValues[3] > irvalue) {
              Serial.print("Soft Right"); 
              motorspeed_A = soft_speed;
        }
        //FORWARD
        else {
              //Serial.print("Right Forward");
              motorspeed_A = straight_speed;   
        }

    //-------------------------------------------
    //MOTOR B
    //-------------------------------------------
        //HARD LEFT
        if (sensorValues[7] > irvalue) {
              Serial.print("Hard Left");
              motorspeed_B = hard_speed; 
        }
        //MODERATE LEFT
        else if (sensorValues[6] > irvalue) {
              Serial.print("Moderate Left");
              motorspeed_B = moderate_speed; 
        }
        //SLIGHT LEFT
        else if (sensorValues[5] > irvalue) {
              Serial.print("Slight Left");
              motorspeed_B = slight_speed; 
        }
        //SOFT LEFT
        else if(sensorValues[4] > irvalue) {
              Serial.print("Soft Left"); 
              motorspeed_B = soft_speed;
        }
        //FORWARD
        else {
              //Serial.print("Left Forward");
              motorspeed_B = straight_speed;   
        }

        drive(false, true, motorspeed_A, motorspeed_B);
        Serial.print('\t');
  }//end read_array



  void drive(bool motor1, bool motor2, int motorspeed1, int motorspeed2) {

 
            digitalWrite (motorA1, motor1);   //L
            digitalWrite(motorA2, motor2);    //H 
            analogWrite (motorAspeed, motorspeed1);

            digitalWrite (motorB1, motor1);   //L
            digitalWrite(motorB2,motor2);     //H
            analogWrite (motorBspeed, motorspeed2);
     
    
  } //end drive
