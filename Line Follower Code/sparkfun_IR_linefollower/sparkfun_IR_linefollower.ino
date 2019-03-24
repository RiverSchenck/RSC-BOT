//PWM SPEED VARIATIONS
  int vSpeed = 125;        // MAX 255
  int turn_speed = 150;    // MAX 255
  int SLOW = 10;    // MAX 255  
  int STOP = 0;            // MAX 255 
  
//MOTOR DRIVER CONNECTIONS  
  const int motorA1      = 8;  
  const int motorA2      = 10; 
  const int motorAspeed  = 9;
  const int motorB1      = 12; 
  const int motorB2      = 13; 
  const int motorBspeed  = 11;

//IR SENSOR CONNECTIONS
int QRE1113_Pin_1 = 2; //connected to digital 2
int QRE1113_Pin_2 = 3; //connected to digital 4

//ULTRASOUND SENSOR CONNECTIONS
int howfar = 5; //CHANGE THIS FOR ULTRASOUND SENSATIVITY IN INCHES
const int pingPin = 7; // Trigger Pin of Ultrasonic Sensor
const int echoPin = 6; // Echo Pin of Ultrasonic Sensor

//VARIABLES
long duration, inches; //ULTRASOUND VARIABLES
int whatway;
unsigned long previousMillis = 0;
long interval = 1000;
  

void setup() {
  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);

  Serial.begin(9600);
  delay(3000);
  
}

void loop() {

  ULTRASOUND();
  int QRE_Value_1 = readQD1();
  int QRE_Value_2 = readQD2();
  unsigned long currentMillis = millis();

  

//------------//FORWARD\\------------
  if(QRE_Value_1 < 2700 && QRE_Value_2 < 2700) {

    whatway = 1; //FOR PRINTING DIRECTION

    digitalWrite (motorA2,HIGH);
    digitalWrite(motorA1,LOW);                       
    digitalWrite (motorB2,HIGH);
    digitalWrite(motorB1,LOW);

    analogWrite (motorAspeed, vSpeed);
    analogWrite (motorBspeed, vSpeed);  
  }

//------------//TURN lEFT\\------------
  if(QRE_Value_1 < 2400 && QRE_Value_2 > 2400) {

    whatway = 2; //FOR PRINTING DIRECTION

    digitalWrite (motorA2,HIGH);
    digitalWrite(motorA1,LOW);                       
    digitalWrite (motorB2,HIGH);
    digitalWrite(motorB1,LOW);

    analogWrite (motorAspeed, SLOW);
    analogWrite (motorBspeed, turn_speed); 
   
  }

//------------//TURN RIGHT\\------------
  if(QRE_Value_1 > 2400 && QRE_Value_2 < 2400) {

    whatway = 3; //FOR PRINTING DIRECTION

    digitalWrite (motorA2,HIGH);
    digitalWrite(motorA1,LOW);                       
    digitalWrite (motorB2,HIGH);
    digitalWrite(motorB1,LOW);

    analogWrite (motorAspeed, turn_speed);
    analogWrite (motorBspeed, SLOW); 
   
  }

  //------------//STOP\\------------
  if(QRE_Value_1 > 2400 && QRE_Value_2 > 2400) {

    whatway = 4; //FOR PRINTING DIRECTION
    
    digitalWrite (motorA2,LOW);
    digitalWrite(motorA1,LOW);                       
    digitalWrite (motorB2,LOW);
    digitalWrite(motorB1,LOW);

    analogWrite (motorAspeed, STOP);
    analogWrite (motorBspeed, STOP); 
   
  }


//PRINT OUT DIRECTION WE ARE GOING
//PRINTS EVERY 1 SECOND

if (currentMillis - previousMillis > interval) {
  previousMillis = currentMillis;  
  Serial.print("Sensor 1:");
  Serial.print(QRE_Value_1); 
  Serial.print(" Sensor 2:");
  Serial.println(QRE_Value_2); 
  switch(whatway) {
    case 1:
       Serial.println("GOING FORWARD");
       break;
    case 2:
        Serial.println("TURNING LEFT");
        break;
    case 3:
        Serial.println("TURNING RIGHT");
        break;
    case 4:
        Serial.println("STOP");
        break;
    
  }

}


}

void ULTRASOUND(){
  do{
    
   pinMode(pingPin, OUTPUT);
   digitalWrite(pingPin, LOW);
   delayMicroseconds(2);
   digitalWrite(pingPin, HIGH);
   delayMicroseconds(10);
   digitalWrite(pingPin, LOW);
   pinMode(echoPin, INPUT);
   duration = pulseIn(echoPin, HIGH);
   inches = duration / 74 / 2;
   delay(100);

    if(inches < howfar) {
        Serial.print(inches);
        Serial.print("in, ");
        Serial.println();
        digitalWrite(LED_BUILTIN, HIGH);
        Serial.println("STOP");
  
        digitalWrite (motorA2,LOW);
        digitalWrite(motorA1,LOW);                       
        digitalWrite (motorB2,LOW);
        digitalWrite(motorB1,LOW);
  
        analogWrite (motorAspeed, STOP);
        analogWrite (motorBspeed, STOP); 
    }
    
  
     digitalWrite(LED_BUILTIN, LOW);
  } while(inches < howfar);

}


int readQD1(){
  //Lower numbers mean more refleacive
  //More than 3000 means nothing was reflected.
  pinMode( QRE1113_Pin_1, OUTPUT );
  digitalWrite( QRE1113_Pin_1, HIGH );
  delayMicroseconds(10);
  pinMode( QRE1113_Pin_1, INPUT );

  long time = micros();

  //time how long the input is HIGH, but quit after 3ms as nothing happens after that
  while (digitalRead(QRE1113_Pin_1) == HIGH && micros() - time < 3000);
  int diff = micros() - time;

  return diff;
}

int readQD2(){
  //Lower numbers mean more refleacive
  //More than 3000 means nothing was reflected.
  pinMode( QRE1113_Pin_2, OUTPUT );
  digitalWrite( QRE1113_Pin_2, HIGH );
  delayMicroseconds(10);
  pinMode( QRE1113_Pin_2, INPUT );

  long time = micros();

  //time how long the input is HIGH, but quit after 3ms as nothing happens after that
  while (digitalRead(QRE1113_Pin_2) == HIGH && micros() - time < 3000);
  int diff2 = micros() - time;

  return diff2;
}

 










 
