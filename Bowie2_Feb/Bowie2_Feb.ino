/*
 * Bowie 2 (orange & green) - February code revision
 * -------------------------------------------------
 * 
 * Erin RobotGrrl for RobotMissions.org
 * CC BY-SA
 * robotmissions.org
 * 
 */


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
#include <Servo.h>
//#include <math.h>
#include "RunningAverage.h"
#include <XBee.h>
#include <MahonyAHRS.h>
#include <MadgwickAHRS.h>
#include "PixyUART.h"

boolean DEBUG = false;


// AHRS ---------------------------------------------

Adafruit_L3GD20_Unified       gyro(20);
Adafruit_LSM303_Accel_Unified accel(30301);
Adafruit_LSM303_Mag_Unified   mag(30302);

// Offsets applied to raw x/y/z values
float mag_offsets[3]            = { 0.1F, 0.1F, 0.1F };

// Soft iron error compensation matrix
float mag_softiron_matrix[3][3] = { { 0.0, 0.0, 0.0 },
                                    { 0.0, 0.0, 0.0 },
                                    { 0.0, 0.0, 0.0 } }; 

float mag_field_strength        = 48.41F;

// Mahony is lighter weight as a filter and should be used
// on slower systems
//Mahony filter;
Madgwick filter;

// averaging filter
#define READ_SAMPLES 10
#define PREV_WEIGHT 0.4
int num_sample = 0;
float heading_reading[2];
float heading_total = 0;
float heading_result = 0;

// go to a target
#define DESTINATION_THRESH 5
float target = 30;
float alpha = 0;
bool turning = false;
bool clockwise = false;

int subs = 0;
long last_sub = 0;

// ---------------------------------------------

// COMMS ---------------------------------------

Promulgate promulgate = Promulgate(&Serial2, &Serial);

void transmit_complete();
void received_action(char action, char cmd, uint8_t key, uint16_t val, char delim);

long last_rx = 0;

int led = 13;
int led_green = 21;
int superbright_l = 20;
int superbright_r = 3;

// ---------------------------------------------

// MOTORS --------------------------------------

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

#define MOTOR_DIR_FWD false
#define MOTOR_DIR_REV true
void motor_init();
void motor_setDir(uint8_t motorNum, bool dir);
void motor_setSpeed(uint8_t motorNum, uint8_t speed);
void motor_setBrake(uint8_t motorNum);
void motor_setCoast(uint8_t motorNum);

// ---------------------------------------------

// SONAR ---------------------------------------

float dist_thresh = 3.0;
int left_trig = 0;
int right_trig = 0;
int both_trig = 0;
long last_left_trig = 0;
long last_right_trig = 0;
long last_both_trig = 0;
int result = 99; // 0 = nothing, 1 = left, 2 = right, 3 = back
int result_n1 = 99; // what it did at the previous step n-1
int result_n2 = 99; // n-2

int sonar_reading_left = 0;
int sonar_reading_right = 0;

// ---------------------------------------------

// ARM -----------------------------------------

#define SERVO_ARM 4
#define SERVO_CLAW 5
#define SERVO_ARM2 6

Servo arm;
Servo claw;
Servo arm2;

boolean claw_state = false;

int arm_max = 170; // standing up
int arm_home = 100; // standing up
int arm_min = 30; // closest to ground

int claw_min = 1500;
int claw_home = 800;
int claw_max = 500;

// ---------------------------------------------

// CONFIG --------------------------------------

boolean autonomous = false;
long last_print = 0;
boolean blink_on = false;
long current_time = 0;

// ---------------------------------------------

// COMPASS NAV ---------------------------------

int mag_error = 0;
int accel_error = 0;
// last calibrated on aug 12
float hardiron_x = 11.74;
float hardiron_y = 0.39;
float hardiron_z = -45.25;

long last_dest_change = 0;
int dest_state = 0;

int straight_counter = 0;
long last_straight_add = 0;
int turn_counter = 0;
long last_turn_add = 0;
int left_counter = 0;
long last_left_add = 0;
int right_counter = 0;
long last_right_add = 0;
int go_state = 1;
int straight_tick = 0;
long staight_start = 0;
boolean locked = true;
boolean locked_dir = false;
boolean on_heading = false;
long heading_start = 0;

// ---------------------------------------------

// GPS NAV -------------------------------------

String lat_buf = "";
String lon_buf = "";
float lat_current = 0.0;
float lon_current = 0.0;
int reading_state = 0;
long last_gps_receive = 0;
int gps_receives = 0;

double goal_thresh = 1.0;
double goal_distance;
double goal_heading;

long timer_change = 0;

// ---------------------------------------------

// STATES --------------------------------------

int REMOTE_OP_STATE = 1;
int HEADING_STATE = 2;
int GPS_STATE = 3;
int TESTING_STATE = 4;
int MAKERFAIREOTT = 5;
int AHRS_TESTING = 6;

int STATE = REMOTE_OP_STATE;

// ---------------------------------------------

// AVERAGES ------------------------------------

RunningAverage testRA1(10);
RunningAverage testRA2(50);
int samples1 = 0;
int samples2 = 0;

long last_sensor_log = 0;

// ---------------------------------------------

// XBEE ----------------------------------------

XBee xbee = XBee();

XBeeAddress64 addr64 = XBeeAddress64(0x00000000, 0x0000ffff);
XBeeAddress64 addr_controller = XBeeAddress64(0x0013A200, 0x40DD9902);
XBeeAddress64 addr_robot = XBeeAddress64(0x0013A200, 0x40D96FC2);
ZBTxStatusResponse txStatus = ZBTxStatusResponse();
ZBRxResponse rx = ZBRxResponse();
char message_tx[64];
char message_rx[64];
uint32_t msg_tx_count = 0;
uint32_t msg_rx_count = 0;
uint32_t msg_tx_err = 0;

// ---------------------------------------------

PixyUART pixy;

void setup() {

  Serial.begin(9600);   // serial port
  Serial1.begin(9600);  // bluetooth
  Serial2.begin(9600);  // xbee
  //Serial3.begin(9600);  // pixy or gps
  pixy.init();

  xbee.setSerial(Serial2);
  xbee.begin(Serial2);

  promulgate.LOG_LEVEL = Promulgate::ERROR_;
  promulgate.set_rx_callback(received_action);
  promulgate.set_tx_callback(transmit_complete);

  Serial.print(F("Bowie is loading...\n"));
  Serial1.print(F("Bowie is loading...\n"));
  
  pinMode(SONAR_RIGHT, INPUT);
  pinMode(SONAR_LEFT, INPUT);

  pinMode(superbright_l, OUTPUT);
  pinMode(superbright_r, OUTPUT);

  //digitalWrite(superbright_l, HIGH);
  //digitalWrite(superbright_r, HIGH);

  pinMode(led_green, OUTPUT);
  digitalWrite(led_green, HIGH);

  arm.attach(SERVO_ARM);
  claw.attach(SERVO_CLAW);
  arm2.attach(SERVO_ARM2);

  arm.write(180-arm_home);
  arm2.write(arm_home);
  claw.writeMicroseconds(claw_home);

  pinMode(led, OUTPUT);
  digitalWrite(led, HIGH);
  
  motor_init();

  lat_buf.reserve(32);
  lon_buf.reserve(32);

  Serial.print(F("Bowie is ready to go\n"));
  
}

void loop() {

  current_time = millis();
  //delay(20);


  //pixyUpdate();


  if(STATE == AHRS_TESTING) {
    compassAHRS();
    delay(10);
  }


  if(STATE == MAKERFAIREOTT) {

    //distanceSensors();   

    while(Serial2.available()) {
      char c = Serial2.read();
      promulgate.organize_message(c);
      Serial << c;
      if(c == '!') Serial << "\n";
    }
  
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

  }


  if(STATE == REMOTE_OP_STATE) {
  
    while(xbeeRead()){ //if(xbeeRead()) {
      for(int i=0; i<rx.getDataLength(); i++) {
        Serial << message_rx[i];
        promulgate.organize_message(message_rx[i]);
      }
    }
  
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
   * P = red button
   * G = green button
   * Y = yellow button
   * B = blue button
   * W = white button
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
  
  if(action == '#') { // going straight

    if(cmd == 'L') { // left motor
      if(key == 1) { // fwd
        digitalWrite(superbright_l, HIGH);
        leftBork();
        motor_setDir(0, MOTOR_DIR_FWD);
        motor_setSpeed(0, val);
      } else if(key == 0) { // bwd
        digitalWrite(superbright_l, LOW);
        leftBork();
        motor_setDir(0, MOTOR_DIR_REV);
        motor_setSpeed(0, val);
      }
    }
  
    if(cmd == 'R') { // right motor
      if(key == 1) { // fwd
        digitalWrite(superbright_r, HIGH);
        motor_setDir(1, MOTOR_DIR_FWD);
        motor_setSpeed(1, val);   
      } else if(key == 0) { // bwd
        digitalWrite(superbright_r, LOW);
        motor_setDir(1, MOTOR_DIR_REV);
        motor_setSpeed(1, val);
      }
    }

    if(cmd == 'P') { // red button
      if(val == 1) {
        motor_setDir(1, MOTOR_DIR_FWD);
        motor_setSpeed(1, 60);
        motor_setDir(0, MOTOR_DIR_FWD);
        motor_setSpeed(0, 60);
      }
    }

    if(cmd == 'G') { // green button
      if(val == 1) {
        motor_setDir(1, MOTOR_DIR_REV);
        motor_setSpeed(1, 60);
        motor_setDir(0, MOTOR_DIR_REV);
        motor_setSpeed(0, 60);
      }
    }

    if(cmd == 'Y') { // yellow button
      if(val == 1) {
        motor_setDir(1, MOTOR_DIR_FWD);
        motor_setSpeed(1, 120);
        motor_setDir(0, MOTOR_DIR_FWD);
        motor_setSpeed(0, 40);
      }
    }

    if(cmd == 'B') { // blue button
      if(val == 1) {
        motor_setDir(1, MOTOR_DIR_FWD);
        motor_setSpeed(1, 40);
        motor_setDir(0, MOTOR_DIR_FWD);
        motor_setSpeed(0, 120);
      }
    }

    if(cmd == 'W') { // white button
      if(val == 1) {
        digitalWrite(superbright_r, HIGH);
        digitalWrite(superbright_l, HIGH);
        delay(500);
        digitalWrite(superbright_r, LOW);
        digitalWrite(superbright_l, LOW);
        delay(500);
      }
    }

  }

  if(action == '@') { // turning

    if(cmd == 'L') { // left motor
      if(key == 1) { // fwd
        digitalWrite(superbright_l, HIGH);
        leftBork();
        motor_setDir(0, MOTOR_DIR_FWD);
        motor_setSpeed(0, val);
      } else if(key == 0) { // bwd
        digitalWrite(superbright_l, LOW);
        leftBork();
        motor_setDir(0, MOTOR_DIR_REV);
        motor_setSpeed(0, val);
      }
    }
  
    if(cmd == 'R') { // right motor
      if(key == 1) { // fwd
        digitalWrite(superbright_r, HIGH);
        motor_setDir(1, MOTOR_DIR_FWD);
        motor_setSpeed(1, val);
      } else if(key == 0) { // bwd
        digitalWrite(superbright_r, LOW);
        motor_setDir(1, MOTOR_DIR_REV);
        motor_setSpeed(1, val);
      }
    }

  }

  if(cmd == 'S') { // arm (data from 0-45)
     
    int the_pos = (int)map(val, 0, 45, arm_min, arm_max);

    if(the_pos < (115+5) && the_pos > (115-5)) {
      the_pos = 140;
    }
      
    arm.write(180-the_pos);
    arm2.write(the_pos);
    
    Serial << "\narm angle: " << the_pos << endl;
    
  }

  if(cmd == 'C') { // claw
     
    int the_pos = (int)map(val, 0, 45, claw_min, claw_max);
    claw.writeMicroseconds(the_pos);

    Serial << "\nclaw angle: " << the_pos << endl;
    
  }

}

void transmit_complete() {
  
}



