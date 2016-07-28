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

// TB6612FNG motor test

#define MOTORA_SPEED 23
#define MOTORB_SPEED 22
#define MOTORA_CTRL2 21
#define MOTORA_CTRL1 20
#define MOTORB_CTRL1 15
#define MOTORB_CTRL2 14

#define SONAR_LEFT A2
#define SONAR_RIGHT A3
#define SONAR_THRESH 20

int sonar_reading_left = 0;
int sonar_reading_right = 0;




#define SERVO_TOP 4
#define SERVO_END 3


Servo top;
Servo claw;

boolean claw_state = false;


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
}


void setup() {

  Serial.begin(9600);
  Serial2.begin(9600);

  promulgate.LOG_LEVEL = Promulgate::ERROR_;
  promulgate.set_rx_callback(received_action);
  promulgate.set_tx_callback(transmit_complete);



  top.attach(4);
  claw.attach(3);


  
  // put your setup code here, to run once:
  motor_init();
//  motor_setSpeed(0, 20);
//  motor_setSpeed(1, 20);
//  motor_setDir(0, MOTOR_DIR_FWD);
//  motor_setDir(1, MOTOR_DIR_FWD);

  pinMode(SONAR_RIGHT, INPUT);
  pinMode(SONAR_LEFT, INPUT);

  //initSensors();
  
}

void loop() {

  //Serial.println(analogRead(SONAR_RIGHT));
  //delay(50);

  //mag_mode();

  
  if(Serial2.available()) {
    char c = Serial2.read();
    promulgate.organize_message(c);
    //Serial << c;
    //if(c == '!') Serial << "\n";
  }

  //delay(100);
  
  sonar_reading_left = analogRead(SONAR_LEFT);
  sonar_reading_right = analogRead(SONAR_RIGHT);

  /*
  if(sonar_reading_left <= SONAR_THRESH || sonar_reading_right <= SONAR_THRESH) {
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
    motor_setSpeed(0, 255);
    motor_setSpeed(1, 255);
    motor_setDir(0, MOTOR_DIR_FWD);
    motor_setDir(1, MOTOR_DIR_FWD);
  }
  */
  

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




void mag_mode() {

  sensors_event_t mag_event;
  sensors_vec_t   orientation;

  float dest = 160.0;
  int the_speed = 255;
  float thresh = 3.0;

  /* Calculate the heading using the magnetometer */
  mag.getEvent(&mag_event);
  if (dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation))
  {
    /* 'orientation' should have valid .heading data now */
    Serial.print(F("Heading: "));
    Serial.print(orientation.heading);
    Serial.print(F("; "));
  }

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
  
}








void received_action(char action, char cmd, uint8_t key, uint16_t val, char delim) {

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

  if(action == '#') {

    if(cmd == 'L') { // left motor
      if(key == 1) { // fwd
        motor_setDir(0, MOTOR_DIR_FWD);
        motor_setSpeed(0, val);
      } else if(key == 0) { // bwd
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

  } else if(action == '@') {

    if(cmd == 'L') { // left motor
      if(key == 1) { // fwd
        motor_setDir(0, MOTOR_DIR_FWD);
        motor_setSpeed(0, val);
      } else if(key == 0) { // bwd
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

      // for the claw arm
      int claw_arm_pos = (int)map(val, 0, 45, 600, 800);
      top.writeMicroseconds(claw_arm_pos);
      
    }
  
    if(cmd == 'C') { // claw
      digitalWrite(led, HIGH);
      if(claw_state) {
        claw.writeMicroseconds(2100);
      } else {
        claw.writeMicroseconds(1200);
      }
      digitalWrite(led, LOW);
      claw_state = !claw_state;
    }
   

  
}

void transmit_complete() {
  
}



