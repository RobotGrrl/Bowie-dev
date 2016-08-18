void followHeading(float dest, int dir) {

  sensors_event_t event_accl;
  sensors_event_t event_magn;

  accel.getEvent(&event_accl);
  mag.getEvent(&event_magn);

  if(event_magn.magnetic.x == 0.0 && event_magn.magnetic.y == 0.0 && event_magn.magnetic.z == 0.0) {
    mag_error++;
    return;
  }

  if(event_accl.acceleration.x == 0.0 && event_accl.acceleration.y == 0.0 && event_accl.acceleration.z == 0.0) {
    accel_error++;
    return;
  }
  
  // code based on snippet from this page
  // http://srlm.io/2014/09/16/experimenting-with-magnetometer-calibration/
 
  // Signs choosen so that, when axis is down, the value is + 1g
  float accl_x = -event_accl.acceleration.x;
  float accl_y = event_accl.acceleration.y;
  float accl_z = event_accl.acceleration.z;
   
  // Signs should be choosen so that, when the axis is down, the value is + positive.
  // But that doesn't seem to work ?...
  float magn_x = event_magn.magnetic.x - hardiron_x;
  float magn_y = -event_magn.magnetic.y - hardiron_y;
  float magn_z = -event_magn.magnetic.z - hardiron_z;


  // code from adafruit 10DOF library
  // Adafruit_10DOF::magGetOrientation with SENSOR_AXIS_Z
  float heading = (float)atan2(magn_y, magn_x) * 180 / PI;
  
  if (heading < 0) {
    heading = 360 + heading;
  }

  /*
  Serial.print(" x: "); Serial.print(event_magn.magnetic.x);
  Serial.print(" y: "); Serial.print(event_magn.magnetic.y);
  Serial.print(" z: "); Serial.print(event_magn.magnetic.z);

  Serial.print(" magn_x: "); Serial.print(magn_x);
  Serial.print(" magn_y: "); Serial.print(magn_y);
  Serial.print(" magn_z: "); Serial.print(magn_z);
  */

  int the_speed = 255;
  float thresh = 15.0;

  // why wasn't this working reliably?
  //float diff = (float)atan2(sin(heading-dest), cos(heading-dest)) * 180 / PI;

  // discrete version
  int diff = (int)dest - (int)heading;
  diff = (diff + 180) % 360 - 180;
  if(heading > (dest+180)) diff = 360+diff;

  Serial.print(" Heading: "); Serial.print(heading);
  Serial.print(" Dest: "); Serial.print(dest);
  Serial.print(" Diff "); Serial.print(diff);
  Serial.print(" ");

  
  if(abs(diff) < thresh) { // go straight
    straight_counter++;
    last_straight_add = millis();
  } else if(diff > 0) { // turn right
    if(!locked_dir) {
      left_counter++;
      last_left_add = millis();
    }
  } else if(diff < 0) { // turn left
    if(!locked_dir) {
      right_counter++;
      last_right_add = millis();
    }
  }
  
  
  if(millis()-last_straight_add >= 2000) {     
    straight_counter = 0;
  }

  if(millis()-last_left_add >= 2000) {
    left_counter = 0;
  }

  if(millis()-last_right_add >= 2000) {
    right_counter = 0;
  }

  if(straight_counter >= 5) {
    straight_counter = 0;
    if(go_state != 0) {
      heading_start = millis();
      Serial.print("\nHeading start: ");
      Serial.print(heading_start);
      Serial.print("\n");
    }
    go_state = 0;
    locked_dir = false;
  }

  if(left_counter >= 5) {                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     
    right_counter = 0;
    go_state = 1;
    locked_dir = true;
  }

  if(right_counter >= 5) {
    right_counter = 0;
    go_state = 2;
    locked_dir = true;
  }

  if(go_state == 0) {
    on_heading = true;
  } else {
    on_heading = false;
  }


  // (in)sanity check:
  // if it's not on auto choice mode, and we're not on the heading cource, and the last time
  // we were on the heading course was more than 3s ago, and it isn't at the beginning,
  // then go with the default turn direction
  /*
  if(dir != 99) {
    if(!on_heading) {
      if(millis()-heading_start >= 3000 && heading_start != 0) {
        go_state = dir;
      }
    }
  }
  */



  switch(go_state) {
    case 0: // straight
    Serial.print("S ");
    digitalWrite(superbright_l, HIGH);
    digitalWrite(superbright_r, HIGH);
    leftBork();
    motor_setDir(1, MOTOR_DIR_FWD);
    motor_setSpeed(1, the_speed);
    motor_setDir(0, MOTOR_DIR_FWD);
    motor_setSpeed(0, the_speed);
    break;
    case 1: // left
    Serial.print("L ");
    digitalWrite(superbright_l, HIGH);
    digitalWrite(superbright_r, LOW);
    leftBork();
    motor_setDir(1, MOTOR_DIR_FWD);
    motor_setSpeed(1, 60);//the_speed);
    motor_setDir(0, MOTOR_DIR_FWD);
    motor_setSpeed(0, the_speed);
    break;
    case 2: // right
    Serial.print("R ");
    digitalWrite(superbright_l, LOW);
    digitalWrite(superbright_r, HIGH);
    leftBork();
    motor_setDir(0, MOTOR_DIR_FWD);
    motor_setSpeed(0, 60);//the_speed);
    motor_setDir(1, MOTOR_DIR_FWD);
    motor_setSpeed(1, the_speed);
    break;
  }
  
  //Serial.print(" Heading: "); Serial.print(heading);
  //Serial.print(" mag errors: "); Serial.print(mag_error);
  //Serial.print(" accel errors: "); Serial.print(accel_error);
  Serial.print("\n");
  delay(250);
  
}

