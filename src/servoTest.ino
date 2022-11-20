#include <ESP32Servo.h>
 
Servo myservo1, myservo2;

int servoPin1 = 25;
int servoPin2 = 27;
 
void setup() {
  // Allow allocation of all timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  myservo1.setPeriodHertz(50);    // standard 50 hz servo
  myservo1.attach(servoPin1, 500, 2400); // attaches the servo on pin 18 to the servo object
  myservo2.setPeriodHertz(50);    // standard 50 hz servo
  myservo2.attach(servoPin2, 500, 2400);
}
 
void loop() {
  myservo1.write(0);
  delay(1000);
  myservo2.write(0);
  delay(1000);
  myservo1.write(45);
  delay(1000);
  myservo2.write(45);
  delay(1000);
  myservo1.write(90);
  delay(1000);
  myservo2.write(90);
  delay(1000);
}