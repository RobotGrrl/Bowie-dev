#include <Servo.h>

int pinA = 4;
int pinB = 5;
int pinC = 6;

Servo servoA;
Servo servoB;
Servo servoC;

void setup() {
  servoA.attach(4);
  servoB.attach(5);
  servoC.attach(6);
}

void loop() {

  for(int i=0; i<180; i++) {
    servoA.write(i);
    servoB.write(i);
    servoC.write(i);
    delay(10);
  }

  for(int i=180; i>0; i--) {
    servoA.write(i);
    servoB.write(i);
    servoC.write(i);
    delay(10);
  }
  
}

