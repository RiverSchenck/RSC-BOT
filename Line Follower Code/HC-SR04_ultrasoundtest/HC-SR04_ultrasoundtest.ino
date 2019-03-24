const int pingPin = 7; // Trigger Pin of Ultrasonic Sensor
const int echoPin = 6; // Echo Pin of Ultrasonic Sensor

void setup() {
   pinMode(LED_BUILTIN, OUTPUT);
   Serial.begin(9600); // Starting Serial Terminal
}

void loop() {
   long duration, inches;
   pinMode(pingPin, OUTPUT);
   digitalWrite(pingPin, LOW);
   delayMicroseconds(2);
   digitalWrite(pingPin, HIGH);
   delayMicroseconds(10);
   digitalWrite(pingPin, LOW);
   pinMode(echoPin, INPUT);
   duration = pulseIn(echoPin, HIGH);
   inches = duration / 74 / 2;
   Serial.print(inches);
   Serial.print("in, ");
   Serial.println();
   delay(100);

   if(inches < 10) {
      digitalWrite(LED_BUILTIN, HIGH);
   }
   else {
      digitalWrite(LED_BUILTIN, LOW);
   }
}
