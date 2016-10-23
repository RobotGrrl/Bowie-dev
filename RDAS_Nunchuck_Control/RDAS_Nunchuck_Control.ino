/*
 *  RDAS Nunchuck Control
 *  ---------------------
 *  Arduino UNO
 *  
 *  xbee rx to 3
 *  xbee tx to 2
 *  nunchuck purple (d) to a4
 *  nunchuck orange (c) to a5
 *  
 *  0 - gnd
 *  1 - 5v
 *  2 - A4
 *  3 - A5
 *  4 - pin 3
 *  5 - pin 2
 * 
 *  Erin RobotGrrl
 *  Jan. 25, 2016
 *  
 *  buttons
 *  red: brown 8
 *  green: red 7
 *  yellow: orange 6
 *  blue: yellow 5
 *  white: green 4
 *  
 *  leds
 *  1: white A3
 *  2: grey A2
 *  3: purple A1
 *  4: blue A0
 * 
 */

#include <Wire.h>
#include <ArduinoNunchuk.h>
#include <SoftwareSerial.h>
#include <Streaming.h>
#include "Promulgate.h"
#include <Bounce2.h>
#include <XBee.h>

boolean DEBUG = false;
boolean MYO_MODE = false;

SoftwareSerial mySerial(2, 3); // RX, TX
Promulgate promulgate = Promulgate(&mySerial, &mySerial);

int led = 13;
boolean led_on = false;
long current_time = 0;
long last_time = 0;
boolean meepmeep = true;

ArduinoNunchuk nunchuk = ArduinoNunchuk();

int max_x = 228;
int min_x = 34;
int max_y = 223;
int min_y = 34;
int home_x = 125;
int home_y = 135;

int tilt_mode = 0;
int max_tilt_modes = 5;
long last_c = 0;
long last_control = 0;
long claw_time = 0;

int read_speed = 0;
int read_dir = 0;

#define REDBUTTON 8
#define GREENBUTTON 7
#define YELLOWBUTTON 6
#define BLUEBUTTON 5
#define WHITEBUTTON 4

#define LED1 A3
#define LED2 A2
#define LED3 A1
#define LED4 A0

#define DEBOUNCE 10

Bounce redbouncer = Bounce();//(REDBUTTON, DEBOUNCE);
Bounce greenbouncer = Bounce();//(GREENBUTTON, DEBOUNCE);
Bounce yellowbouncer = Bounce();//(YELLOWBUTTON, DEBOUNCE);
Bounce bluebouncer = Bounce();//(BLUEBUTTON, DEBOUNCE);
Bounce whitebouncer = Bounce();//(WHITEBUTTON, DEBOUNCE);

// ------ Xbee variables
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



void setup() {
  Serial.begin(9600);
  mySerial.begin(9600);
  xbee.begin(mySerial);
  
  Serial.println("Bowie Nunchuck Control");
  
  promulgate.LOG_LEVEL = Promulgate::ERROR_;
  promulgate.set_rx_callback(received_action);
  promulgate.set_tx_callback(transmit_complete);

  pinMode(led, OUTPUT);

  pinMode(REDBUTTON, INPUT);
  pinMode(GREENBUTTON, INPUT);
  pinMode(YELLOWBUTTON, INPUT);
  pinMode(BLUEBUTTON, INPUT);
  pinMode(WHITEBUTTON, INPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(LED4, OUTPUT);

  redbouncer.attach(REDBUTTON);
  redbouncer.interval(DEBOUNCE);

  greenbouncer.attach(GREENBUTTON);
  greenbouncer.interval(DEBOUNCE);

  yellowbouncer.attach(YELLOWBUTTON);
  yellowbouncer.interval(DEBOUNCE);

  bluebouncer.attach(BLUEBUTTON);
  bluebouncer.interval(DEBOUNCE);

  whitebouncer.attach(WHITEBUTTON);
  whitebouncer.interval(DEBOUNCE);

/*
  while(1<3) {

//    digitalWrite(LED1, HIGH);
//    digitalWrite(LED2, HIGH);
//    digitalWrite(LED3, HIGH);
//    digitalWrite(LED4, HIGH);
//    delay(500);
    digitalWrite(LED1, LOW);
    digitalWrite(LED2, LOW);
    digitalWrite(LED3, LOW);
    digitalWrite(LED4, LOW);
    //delay(500);
    
    redbouncer.update();
    greenbouncer.update();
    yellowbouncer.update();
    bluebouncer.update();
    whitebouncer.update();

    if(redbouncer.read() == HIGH) digitalWrite(LED1, HIGH);
    if(greenbouncer.read() == HIGH) digitalWrite(LED2, HIGH);
    if(yellowbouncer.read() == HIGH) digitalWrite(LED3, HIGH);
    if(bluebouncer.read() == HIGH) digitalWrite(LED4, HIGH);
    if(whitebouncer.read() == HIGH) {
      digitalWrite(LED1, HIGH);
      digitalWrite(LED2, HIGH);
      digitalWrite(LED3, HIGH);
      digitalWrite(LED4, HIGH);
    }
    
  }
  */
  
  nunchuk.init();
}

void loop() {

  current_time = millis();

  if(led_on) {
    digitalWrite(led, LOW);
  } else {
    digitalWrite(led, HIGH);
  }
  led_on = !led_on;

  /*  
  // nunchuck debug print stuff
  Serial.print(nunchuk.analogX, DEC);
  Serial.print(' ');
  Serial.print(nunchuk.analogY, DEC);
  Serial.print(' ');
  Serial.print(nunchuk.accelX, DEC);
  Serial.print(' ');
  Serial.print(nunchuk.accelY, DEC);
  Serial.print(' ');
  Serial.print(nunchuk.accelZ, DEC);
  Serial.print(' ');
  Serial.print(nunchuk.zButton, DEC);
  Serial.print(' ');
  Serial.println(nunchuk.cButton, DEC);

  delay(100);
  */

  nunchuk.update();
    
  if(nunchuk.zButton == 0 && nunchuk.cButton == 0) { // drive

    int motor_speed = 0;
    boolean motor_dir = true;

    if(nunchuk.analogY > max_y) nunchuk.analogY = max_y;
    if(nunchuk.analogY < min_y) nunchuk.analogY = min_y;
    if(nunchuk.analogX > max_x) nunchuk.analogX = max_x;
    if(nunchuk.analogX < min_x) nunchuk.analogX = min_x;

    boolean blorp = false;

    if(nunchuk.analogY >= (home_y-10) && nunchuk.analogY <= (home_y+10)
       && nunchuk.analogX >= (home_x-10) && nunchuk.analogX <= (home_x+10)) {
      
      // stand still
      xbeeSend('#', 'L', 1, 0, '!');
      xbeeSend('#', 'R', 1, 0, '!');
      //promulgate.transmit_action('#', 'L', 1, 0, '!');
      //promulgate.transmit_action('#', 'R', 1, 0, '!');
      
    } else if(nunchuk.analogY >= (home_y-10) && nunchuk.analogY <= (home_y+10)) { // turning
      
      if(nunchuk.analogX >= (min_x+10)) {
        xbeeSend('@', 'L', 0, 255, '!');
        xbeeSend('@', 'R', 1, 255, '!');
        //promulgate.transmit_action('@', 'L', 0, 255, '!');
        //promulgate.transmit_action('@', 'R', 1, 255, '!');
      }
      if(nunchuk.analogX <= (max_x-10)) {
        xbeeSend('@', 'L', 1, 255, '!');
        xbeeSend('@', 'R', 0, 255, '!');
        //promulgate.transmit_action('@', 'L', 1, 255, '!');
        //promulgate.transmit_action('@', 'R', 0, 255, '!');
      }
      
    } else if(nunchuk.analogY >= home_y) { // fwd
      
      motor_speed = map(nunchuk.analogY, home_y, max_y, 0, 255);
      motor_dir = true;
      blorp = true;
      
    } else { // bwd
      
      motor_speed = map(nunchuk.analogY, min_y, home_y, 255, 0);
      motor_dir = false;
      blorp = true;
      
    }

    if(blorp) { // calculate the motor speed for L and R
      
      float percent_r = (float)map(nunchuk.analogX, min_x, max_x, 0, 100);
      percent_r /= 100.0;
      float percent_l = 1.0-percent_r;
  
      int speed_r = (int)((float)motor_speed * percent_r);
      int speed_l = (int)((float)motor_speed * percent_l);
  
      if(DEBUG) Serial << "speed L: " << speed_l << " R: " << speed_r << endl;
      
      // sending the data
      if(motor_dir) {
        xbeeSend('#', 'L', 1, motor_speed, '!');
        xbeeSend('#', 'R', 1, motor_speed, '!');
        //promulgate.transmit_action('#', 'L', 1, motor_speed, '!');
        //promulgate.transmit_action('#', 'R', 1, motor_speed, '!');
      } else {
        xbeeSend('#', 'L', 0, motor_speed, '!');
        xbeeSend('#', 'R', 0, motor_speed, '!');
        //promulgate.transmit_action('#', 'L', 0, motor_speed, '!');
        //promulgate.transmit_action('#', 'R', 0, motor_speed, '!');
      }

    }

    if(DEBUG) {
      if(motor_dir) {
        Serial << "FWD- ";
      } else {
        Serial << "BWD- ";
      }
    
      Serial << "motor_speed: " << motor_speed << endl;
    }
    
  } else if(nunchuk.zButton == 1 && nunchuk.cButton == 0) { // arm

    int servo_pos = map(nunchuk.analogY, min_y, max_y, 0, 45);

    xbeeSend('#', 'S', 0, servo_pos, '!');
    //promulgate.transmit_action('#', 'S', 0, servo_pos, '!');

    Serial << "arm pos: " << servo_pos << endl;
    
  } else if(nunchuk.zButton == 0 && nunchuk.cButton == 1) {

    int servo_pos = map(nunchuk.analogY, min_y, max_y, 0, 45);

    xbeeSend('#', 'C', 0, servo_pos, '!');
    //promulgate.transmit_action('#', 'C', 0, servo_pos, '!');

    Serial << "claw pos: " << servo_pos << endl;
    
  }

  delay(50);

}



void received_action(char action, char cmd, uint8_t key, uint16_t val, char delim) {
  
  if(DEBUG) {
    Serial << "---CALLBACK---" << endl;
    Serial << "action: " << action << endl;
    Serial << "command: " << cmd << endl;
    Serial << "key: " << key << endl;
    Serial << "val: " << val << endl;
    Serial << "delim: " << delim << endl;
  }

  if(cmd == 'G') {
    int soil_reading = val;
    //Serial << "Soil, " << soil_reading << endl;
  }
  
}

void transmit_complete() {
  if(DEBUG) Serial << "transmit complete!" << endl;
}



