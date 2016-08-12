
// note 2: i don't know why the motor a needs to have the pins redo the mode
// thing. but it makes it work. previously it would get 'stuck' going from
// fwd to rev. double checked the wiring, so not sure.


#include <Streaming.h>
#include "Promulgate.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_10DOF.h>
#include <Servo.h>

/* Assign a unique ID to the sensors */
Adafruit_10DOF                dof   = Adafruit_10DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);

/* Update this with the correct SLP for accurate altitude measurements */
float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;




boolean DEBUG = false;

Promulgate promulgate = Promulgate(&Serial2, &Serial2);


void transmit_complete();
void received_action(char action, char cmd, uint8_t key, uint16_t val, char delim);

int led = 13;
int led_green = 21;
int superbright_l = 20;
int superbright_r = 3;

// TB6612FNG motor test
#define MOTORA_SPEED 23
#define MOTORB_SPEED 22

#define MOTORA_CTRL2 17
#define MOTORA_CTRL1 16

#define MOTORB_CTRL1 12
#define MOTORB_CTRL2 11

#define SONAR_LEFT A10
#define SONAR_RIGHT A11
#define SONAR_THRESH 20

int sonar_reading_left = 0;
int sonar_reading_right = 0;




#define SERVO_ARM 4
#define SERVO_CLAW 5

Servo arm;
Servo claw;

boolean claw_state = false;

int arm_max = 1000;
int arm_home = 600;
int arm_min = 400;

int claw_min = 1300;
int claw_home = 800;
int claw_max = 500;


long last_print = 0;
boolean blink_on = false;

boolean fwd_l = true;

boolean autonomous = false;


long last_rx = 0;

int mag_error = 0;


void motor_init() {
  pinMode(MOTORA_CTRL1, OUTPUT);
  digitalWrite(MOTORA_CTRL1, LOW);
  pinMode(MOTORA_CTRL2, OUTPUT);
  digitalWrite(MOTORA_CTRL2, LOW);
  pinMode(MOTORA_SPEED, OUTPUT);
  analogWrite(MOTORA_SPEED, 0);

  pinMode(MOTORB_CTRL1, OUTPUT);
  digitalWrite(MOTORB_CTRL1, LOW);
  pinMode(MOTORB_CTRL2, OUTPUT);
  digitalWrite(MOTORB_CTRL2, LOW);
  pinMode(MOTORB_SPEED, OUTPUT);
  analogWrite(MOTORB_SPEED, 0);
}

#define MOTOR_DIR_FWD false
#define MOTOR_DIR_REV true

void motor_setDir(uint8_t motorNum, bool dir) {
  if(0 == motorNum) {
    if(dir) {
      digitalWrite(MOTORA_CTRL1, HIGH);
      digitalWrite(MOTORA_CTRL2, LOW);        
    } else {
      digitalWrite(MOTORA_CTRL1, LOW);
      digitalWrite(MOTORA_CTRL2, HIGH);
    }
  } else if(1 == motorNum) {
    if(dir) {
      digitalWrite(MOTORB_CTRL2, HIGH);
      digitalWrite(MOTORB_CTRL1, LOW);    
    } else {
      digitalWrite(MOTORB_CTRL2, LOW);
      digitalWrite(MOTORB_CTRL1, HIGH);    
    }    
  }
}

void motor_setSpeed(uint8_t motorNum, uint8_t speed) {
  if(0 == motorNum) {
    analogWrite(MOTORA_SPEED, speed);
  } else if(1 == motorNum) {
    analogWrite(MOTORB_SPEED, speed);
  }
}

void motor_setBrake(uint8_t motorNum) {
  motor_setSpeed(motorNum, 0);
}

void motor_setCoast(uint8_t motorNum) {
  if(0 == motorNum) {
    digitalWrite(MOTORA_CTRL1, LOW);
    digitalWrite(MOTORA_CTRL2, LOW);
    digitalWrite(MOTORA_SPEED, HIGH);
  } else if(1 == motorNum) {
    digitalWrite(MOTORB_CTRL1, LOW);
    digitalWrite(MOTORB_CTRL2, LOW);
    digitalWrite(MOTORB_SPEED, HIGH);
  }
}


void initSensors()
{
  if(!accel.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while(1);
  }
  if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
  if(!bmp.begin())
  {
    /* There was a problem detecting the BMP180 ... check your connections */
    Serial.println("Ooops, no BMP180 detected ... Check your wiring!");
    while(1);
  }

  mag.enableAutoRange(true);
  mag.setMagRate(LSM303_MAGRATE_30);
  
}


void setup() {

  delay(2000);
  
  Serial.begin(9600);
  Serial2.begin(9600);

  Serial << "Hello! I am Bowie!\n";

  promulgate.LOG_LEVEL = Promulgate::ERROR_;
  promulgate.set_rx_callback(received_action);
  promulgate.set_tx_callback(transmit_complete);
  
  pinMode(SONAR_RIGHT, INPUT);
  pinMode(SONAR_LEFT, INPUT);

  pinMode(superbright_l, OUTPUT);
  pinMode(superbright_r, OUTPUT);

  pinMode(led_green, OUTPUT);
  digitalWrite(led_green, HIGH);

  Serial.println("Hellooooooooooooooo");

  arm.attach(SERVO_ARM);
  claw.attach(SERVO_CLAW);

  arm.writeMicroseconds(arm_home);
  claw.writeMicroseconds(claw_home);

  pinMode(led, OUTPUT);
  digitalWrite(led, HIGH);
  
  
  motor_init();

  
//  while(1<3) {
//    motor_setSpeed(0, 255);
//    motor_setSpeed(1, 255);
//    motor_setDir(0, MOTOR_DIR_FWD);
//    motor_setDir(1, MOTOR_DIR_FWD);
//  }

//  while(1<3){
//  motor_setDir(0, MOTOR_DIR_REV);
//  motor_setDir(1, MOTOR_DIR_REV);
//  for(int i=0; i<250; i++) {
//    motor_setSpeed(0, i);
//    motor_setSpeed(1, i);
//    delay(10);
//  }
//  for(int i=250; i>0; i--) {
//    motor_setSpeed(0, i);
//    motor_setSpeed(1, i);
//    delay(10);
//  }
//  motor_setDir(0, MOTOR_DIR_FWD);
//  motor_setDir(1, MOTOR_DIR_FWD);
//  for(int i=0; i<250; i++) {
//    motor_setSpeed(0, i);
//    motor_setSpeed(1, i);
//    delay(10);
//  }
//  for(int i=250; i>0; i--) {
//    motor_setSpeed(0, i);
//    motor_setSpeed(1, i);
//    delay(10);
//  }
//  }


//while(1<3) {
//motor_setDir(0, MOTOR_DIR_FWD);
//motor_setSpeed(0, 60);
//delay(20);
//motor_setDir(0, MOTOR_DIR_REV);
//motor_setSpeed(0, 60);
//delay(20);  
////}
//
//while(1<3)  {
//
//motor_setSpeed(0, 60);
//    motor_setSpeed(1, 60);
//    motor_setDir(0, MOTOR_DIR_FWD);
//    motor_setDir(1, MOTOR_DIR_REV);
//  
//}




  //arm.writeMicroseconds(400); // higher val = up, lower val = down
  //claw.writeMicroseconds(500); // higher val = down, lower val = up
//
//while(1<3) {
//
//  for(int i=arm_min; i<arm_max; i++) {
//    arm.writeMicroseconds(i);
//    delay(1);
//  }
//
//  for(int i=claw_min; i>claw_max; i--) {
//    claw.writeMicroseconds(i);
//    delay(1);
//  }
//
//  for(int i=arm_max; i>arm_min; i--) {
//    arm.writeMicroseconds(i);
//    delay(1);
//  }
//
//  for(int i=claw_max; i<claw_min; i++) {
//    claw.writeMicroseconds(i);
//    delay(1);
//  }
//  
//}

  initSensors();

}

void loop() {

  /*
  motor_setSpeed(0, 255);
  motor_setSpeed(1, 255);
  motor_setDir(0, MOTOR_DIR_FWD);
  motor_setDir(1, MOTOR_DIR_FWD);
  */

  /*
  if(millis()-last_print >= 100) {
    Serial.print("~");
    if(blink_on) {
      digitalWrite(led, HIGH);
    } else {
      digitalWrite(led, LOW);
    }
    blink_on = !blink_on;
    last_print = millis();
  }
  */

  mag_calibrate();

  mag_mode();

  
  if(Serial2.available()) {
    char c = Serial2.read();
    promulgate.organize_message(c);
    Serial << c;
    if(c == '!') Serial << "\n";
  }


/*
  if(millis()-last_rx >= 500) {
    digitalWrite(led_green, LOW);
    leftBork();
    motor_setDir(0, MOTOR_DIR_FWD);
    motor_setSpeed(0, 0);
    motor_setDir(1, MOTOR_DIR_FWD);
    motor_setSpeed(1, 0);
  } else {
    digitalWrite(led_green, HIGH);
  }
  */

  //delay(100);

  /*
  sonar_reading_left = analogRead(SONAR_LEFT);
  sonar_reading_right = analogRead(SONAR_RIGHT);

  if(sonar_reading_left <= SONAR_THRESH || sonar_reading_right <= SONAR_THRESH) {
    autonomous = true;
    motor_setSpeed(0, 255);
    motor_setSpeed(1, 255);
    motor_setDir(0, MOTOR_DIR_REV);
    motor_setDir(1, MOTOR_DIR_REV);
    delay(1000);
    motor_setSpeed(0, 255);
    motor_setSpeed(1, 255);
    motor_setDir(0, MOTOR_DIR_FWD);
    motor_setDir(1, MOTOR_DIR_REV);
    delay(500);
  } else {
    autonomous = false;
  }
  */
  
  
}




void mag_mode() {

  sensors_event_t event;
  sensors_vec_t   orientation;

  float dest = 360.0;
  int the_speed = 255;
  float thresh = 15.0;
  float boundA = 888.0;
  float boundB = 888.0;
  int dir = 0; // 0 = fwd, 1 = left, 2 = right

  if(dest+thresh > 360) {
    boundA = (dest+thresh)-360.0;
  } else {
    boundA = dest+thresh;
  }

  if(dest-thresh < 0.0) {
    boundB = 360.0-(thresh-dest);
  } else {
    boundB = dest-thresh;
  }



  mag.getEvent(&event);

  if(event.magnetic.x == 0.0 && event.magnetic.y == 0.0 && event.magnetic.z == 0.0) {
    mag_error++;
    return;
  }

  Serial.print("X: "); Serial.print(event.magnetic.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.magnetic.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.magnetic.z); Serial.print("  ");Serial.print("uT");
  Serial.print(" & errors: "); Serial.print(mag_error); Serial.println();
  
  
  if (dof.magGetOrientation(SENSOR_AXIS_Z, &event, &orientation))
  {
    Serial.print(F("Heading: "));
    Serial.print(orientation.heading);
    Serial.print(F("; \n"));
  }

  delay(50);
  
  /*
  if(orientation.heading > boundA) { // go left
    digitalWrite(superbright_l, LOW);
    digitalWrite(superbright_r, HIGH);
    leftBork();
    motor_setDir(1, MOTOR_DIR_FWD);
    motor_setSpeed(1, the_speed);
    motor_setDir(0, MOTOR_DIR_FWD);
    motor_setSpeed(0, 60);//the_speed);
  } else if(orientation.heading < boundB) { // go right
    digitalWrite(superbright_l, HIGH);
    digitalWrite(superbright_r, LOW);
    leftBork();
    motor_setDir(1, MOTOR_DIR_FWD);
    motor_setSpeed(1, 60);//the_speed);
    motor_setDir(0, MOTOR_DIR_FWD);
    motor_setSpeed(0, the_speed);
  } else { // go straight
    digitalWrite(superbright_l, HIGH);
    digitalWrite(superbright_r, HIGH);
    leftBork();
    motor_setDir(1, MOTOR_DIR_FWD);
    motor_setSpeed(1, the_speed);
    motor_setDir(0, MOTOR_DIR_FWD);
    motor_setSpeed(0, the_speed);
  }
  */


  /*
  if(orientation.heading < (dest-thresh)) {
    // go right
    motor_setDir(1, MOTOR_DIR_FWD);
    motor_setSpeed(1, 0);
    motor_setDir(0, MOTOR_DIR_FWD);
    motor_setSpeed(0, the_speed);
  } else if(orientation.heading > (dest+thresh)) {
    // go left
    motor_setDir(1, MOTOR_DIR_FWD);
    motor_setSpeed(1, the_speed);
    motor_setDir(0, MOTOR_DIR_FWD);
    motor_setSpeed(0, 0);
  } else {
    // go straight
    motor_setDir(1, MOTOR_DIR_FWD);
    motor_setSpeed(1, the_speed);
    motor_setDir(0, MOTOR_DIR_FWD);
    motor_setSpeed(0, the_speed);
  }
  */
  
}




void mag_calibrate() {

  float mag_max[] = {0, 0, 0};
  float mag_min[] = {0, 0, 0};
  float mag_offset[] = {0, 0, 0};
  boolean done = false;
  int count = 0;
  boolean skip = false;
  boolean go_left = false;
  mag_error = 0;

  sensors_event_t event;
  sensors_vec_t   orientation;


  Serial.println("Get ready to calibrate in 10s... (will blink 5x)");
  for(int i=0; i<5; i++) {
    digitalWrite(superbright_l, LOW);
    digitalWrite(superbright_r, LOW);
    delay(1000);
    digitalWrite(superbright_l, HIGH);
    digitalWrite(superbright_r, HIGH);
    delay(1000);
  }


  mag.getEvent(&event);

  if(event.magnetic.x == 0.0 && event.magnetic.y == 0.0 && event.magnetic.z == 0.0) {
    mag_error++;
    return;
  }

  mag_min[0] = event.magnetic.x;
  mag_min[1] = event.magnetic.y;
  mag_min[2] = event.magnetic.z;

  mag_max[0] = event.magnetic.x;
  mag_max[1] = event.magnetic.y;
  mag_max[2] = event.magnetic.z;


  if(go_left) {
    // go left
    leftBork();
    motor_setDir(1, MOTOR_DIR_FWD);
    motor_setSpeed(1, 255);
    motor_setDir(0, MOTOR_DIR_REV);
    motor_setSpeed(0, 255);
  } else {
    // go right
    leftBork();
    motor_setDir(1, MOTOR_DIR_REV);
    motor_setSpeed(1, 255);
    motor_setDir(0, MOTOR_DIR_FWD);
    motor_setSpeed(0, 255);
  }


  while(count < 100) {

    skip = false;
    mag.getEvent(&event);

    if(event.magnetic.x == 0.0 && event.magnetic.y == 0.0 && event.magnetic.z == 0.0) {
      mag_error++;
      skip = true;
    }

    if(!skip) {

      if(event.magnetic.x < mag_min[0]) mag_min[0] = event.magnetic.x;
      if(event.magnetic.y < mag_min[1]) mag_min[1] = event.magnetic.y;
      if(event.magnetic.z < mag_min[2]) mag_min[2] = event.magnetic.z;
  
      if(event.magnetic.x > mag_max[0]) mag_max[0] = event.magnetic.x;
      if(event.magnetic.y > mag_max[1]) mag_max[1] = event.magnetic.y;
      if(event.magnetic.z > mag_max[2]) mag_max[2] = event.magnetic.z;
  
      Serial.print("X: "); Serial.print(event.magnetic.x); Serial.print("  ");
      Serial.print("Y: "); Serial.print(event.magnetic.y); Serial.print("  ");
      Serial.print("Z: "); Serial.print(event.magnetic.z); Serial.print("  ");Serial.print("uT");
  
      if (dof.magGetOrientation(SENSOR_AXIS_Z, &event, &orientation)) {
        Serial.print("   Heading: ");
        Serial.print(orientation.heading);
        Serial.print("; \n");
      }
  
      count++;
      delay(100);

    }
    
  }
  

  done = true;

  // stop the motors
  leftBork();
  motor_setDir(1, MOTOR_DIR_FWD);
  motor_setSpeed(1, 0);
  motor_setDir(0, MOTOR_DIR_FWD);
  motor_setSpeed(0, 0);

  for(int i=0; i<3; i++) {
    mag_offset[i] = (mag_min[i] + mag_max[i]) / 2;
  }
  
  Serial.print("\n\n------------ RESULTS -------------\n");
  
  Serial.print("Min ");
  Serial.print("X: "); Serial.print(mag_min[0]); Serial.print("  ");
  Serial.print("Y: "); Serial.print(mag_min[1]); Serial.print("  ");
  Serial.print("Z: "); Serial.print(mag_min[2]); Serial.print("  "); Serial.print("uT\n");

  Serial.print("Max ");
  Serial.print("X: "); Serial.print(mag_max[0]); Serial.print("  ");
  Serial.print("Y: "); Serial.print(mag_max[1]); Serial.print("  ");
  Serial.print("Z: "); Serial.print(mag_max[2]); Serial.print("  "); Serial.print("uT\n");

  Serial.print("Offset ");
  Serial.print("X: "); Serial.print(mag_offset[0]); Serial.print("  ");
  Serial.print("Y: "); Serial.print(mag_offset[1]); Serial.print("  ");
  Serial.print("Z: "); Serial.print(mag_offset[2]); Serial.print("  "); Serial.print("uT\n");

  Serial.print("Num errors: "); Serial.print(mag_error); Serial.println("\n\n");


  Serial.print("\n\n---------- RESULTS (csv) -----------\n");
  Serial.println("Dir, Min X, Min Y, Min Z, Max X, Max Y, Max Z, Off X, Off Y, Off Z");
  if(go_left) {
    Serial.print("0");
  } else {
    Serial.print("1");
  }
  Serial.print(", ");
  Serial.print(mag_min[0]); Serial.print(", ");
  Serial.print(mag_min[1]); Serial.print(", ");
  Serial.print(mag_min[2]); Serial.print(", ");

  Serial.print(mag_max[0]); Serial.print(", ");
  Serial.print(mag_max[1]); Serial.print(", ");
  Serial.print(mag_max[2]); Serial.print(", ");

  Serial.print(mag_offset[0]); Serial.print(", ");
  Serial.print(mag_offset[1]); Serial.print(", ");
  Serial.print(mag_offset[2]); 
  Serial.println("\n");

  
  while(true) {
    digitalWrite(superbright_l, LOW);
    digitalWrite(superbright_r, LOW);
    delay(500);
    digitalWrite(superbright_l, HIGH);
    digitalWrite(superbright_r, HIGH);
    delay(500);
  }

}




void motorTest() {
  
  /*
  motor_setDir(0, MOTOR_DIR_FWD);
  motor_setSpeed(0, 1);
  motor_setBrake(1);
  delay(1000);

  motor_setBrake(0);
  motor_setBrake(1);
  delay(1000);

  motor_setDir(0, MOTOR_DIR_REV);
  motor_setSpeed(0, 1);
  motor_setBrake(1);
  delay(1000);

  motor_setBrake(0);
  motor_setBrake(1);
  delay(1000);

  motor_setDir(1, MOTOR_DIR_FWD);
  motor_setSpeed(1, 200);
  motor_setBrake(0);
  delay(1000);

  motor_setBrake(0);
  motor_setBrake(1);
  delay(1000);

  motor_setDir(1, MOTOR_DIR_REV);
  motor_setSpeed(1, 200);
  motor_setBrake(0);
  delay(1000);

  motor_setBrake(0);
  motor_setBrake(1);
  delay(1000);
  */
  
}


void leftBork() {
  pinMode(MOTORA_CTRL1, OUTPUT);
  digitalWrite(MOTORA_CTRL1, LOW);
  pinMode(MOTORA_CTRL2, OUTPUT);
  digitalWrite(MOTORA_CTRL2, LOW);
  pinMode(MOTORA_SPEED, OUTPUT);
  analogWrite(MOTORA_SPEED, 0);
}


void received_action(char action, char cmd, uint8_t key, uint16_t val, char delim) {

  if(autonomous) return; // override the human

  /* API
   * ----
   * L = left motor
   * R = right motor
   * S = arm
   * T = tilt / center of gravity
   * Z = soil sample leds
   * C = claw
   */
  
  if(DEBUG) {
    Serial << "---CALLBACK---" << endl;
    Serial << "action: " << action << endl;
    Serial << "command: " << cmd << endl;
    Serial << "key: " << key << endl;
    Serial << "val: " << val << endl;
    Serial << "delim: " << delim << endl;
  }

  last_rx = millis();

  if(action == '#') {

    if(cmd == 'L') { // left motor
      if(key == 1) { // fwd
        Serial << "A" << endl;
        leftBork();
        motor_setDir(0, MOTOR_DIR_FWD);
        motor_setSpeed(0, val);
        fwd_l = true;
      } else if(key == 0) { // bwd
        Serial << "B" << endl;
        leftBork();
        motor_setDir(0, MOTOR_DIR_REV);
        motor_setSpeed(0, val);
      }
    }
  
    if(cmd == 'R') { // right motor
      if(key == 1) { // fwd
        Serial << "C" << endl;
        motor_setDir(1, MOTOR_DIR_FWD);
        motor_setSpeed(1, val);   
      } else if(key == 0) { // bwd
        Serial << "D" << endl;
        motor_setDir(1, MOTOR_DIR_REV);
        motor_setSpeed(1, val);
      }
    }

  }

  if(action == '@') {

    if(cmd == 'L') { // left motor
      if(key == 1) { // fwd
        leftBork();
        motor_setDir(0, MOTOR_DIR_FWD);
        motor_setSpeed(0, val);
      } else if(key == 0) { // bwd
        leftBork();
        motor_setDir(0, MOTOR_DIR_REV);
        motor_setSpeed(0, val);
      }
    }
  
    if(cmd == 'R') { // right motor
      if(key == 1) { // fwd
        motor_setDir(1, MOTOR_DIR_FWD);
        motor_setSpeed(1, val);
      } else if(key == 0) { // bwd
        motor_setDir(1, MOTOR_DIR_REV);
        motor_setSpeed(1, val);
      }
    }

  }

  if(cmd == 'S') { // arm (data from 0-45)
     
    int the_pos = (int)map(val, 0, 45, arm_min, arm_max);
    arm.writeMicroseconds(the_pos);
  }

  if(cmd == 'C') { // claw
     
    int the_pos = (int)map(val, 0, 45, claw_min, claw_max);
    claw.writeMicroseconds(the_pos);
  }
   
}

void transmit_complete() {
  
}



