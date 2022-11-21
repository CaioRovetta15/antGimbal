#include <Servo.h>

#include <dummy.h>
//#include <ESP32Servo.h>
 
Servo myservo1, myservo2;

int servoPin1 = 25;
int servoPin2 = 27;
 
void setup() {
  Serial.begin(9600);
  myservo1.attach(servoPin1);
  myservo2.attach(servoPin2);
}
 
void loop() {
  Serial.print("hello");
  myservo1.write(20);
  delay(2000);
  myservo2.write(20);
  delay(2000);
  myservo1.write(45);
  delay(2000);
  myservo2.write(45);
  delay(2000);
  myservo1.write(60);
  delay(2000);
  myservo2.write(60);
  delay(2000);
}