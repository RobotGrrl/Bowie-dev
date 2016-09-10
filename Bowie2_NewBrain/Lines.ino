void followHeading(float dest, int dir) {

  float heading = getCurrentHeading();
  if(heading == 999.99) return;

  bool PRINT = true;

  /*
  Serial.print(" x: "); Serial.print(event_magn.magnetic.x);
  Serial.print(" y: "); Serial.print(event_magn.magnetic.y);
  Serial.print(" z: "); Serial.print(event_magn.magnetic.z);

  Serial.print(" magn_x: "); Serial.print(magn_x);
  Serial.print(" magn_y: "); Serial.print(magn_y);
  Serial.print(" magn_z: "); Serial.print(magn_z);
  */

  int the_speed = 255;
  float thresh = 30.0;

  // why wasn't this working reliably?
  //float diff = (float)atan2(sin(heading-dest), cos(heading-dest)) * 180 / PI;

  // discrete version
  int diff = (int)dest - (int)heading;
  diff = (diff + 180) % 360 - 180;
  if(heading > (dest+180)) diff = 360+diff;

  if(PRINT) Serial.print(" Heading: ");
  if(PRINT) Serial.print(heading);
  if(PRINT) Serial.print(" Dest: ");
  if(PRINT) Serial.print(dest);
  if(PRINT) Serial.print(" Diff ");
  if(PRINT) Serial.print(diff);
  if(PRINT) Serial.print(" ");
  
  /*
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
  */

  float diff_half = ((float)diff)/2;
  float thresh_half = ((float)thresh)/2;

  //Serial << "\n diff half: " << diff_half << " thresh half: " << thresh_half << endl;
  delay(100);

  Serial << "\n";

  if(diff > 0 && float(abs(diff)) > thresh_half) {
    Serial << "turn left\n";
    if(!locked_dir) {
      left_counter++;
      last_left_add = millis();
    }
  } else if(diff < 0 && float(abs(diff)) > thresh_half) {
    Serial << "turn right\n"; 
    if(!locked_dir) {
      right_counter++;
      last_right_add = millis();
    }
  } else {
    Serial << "go fwd\n";
    straight_counter++;
    last_straight_add = millis();
  }

  
  /*
  if(diff_half > thresh_half) { // turn left
    if(!locked_dir) {
      left_counter++;
      last_left_add = millis();
    }
  } else if(diff_half < thresh_half) { // turn right
    if(!locked_dir) {
      right_counter++;
      last_right_add = millis();
    }
  } else { // go straight
    straight_counter++;
    last_straight_add = millis();
  }
  */
  
  
  if(millis()-last_straight_add >= 1000) {     
    straight_counter = 0;
  }

  if(millis()-last_left_add >= 1000) {
    left_counter = 0;
  }

  if(millis()-last_right_add >= 1000) {
    right_counter = 0;
  }

  if(straight_counter >= 5) {
    straight_counter = 0;
    if(go_state != 0) {
      heading_start = millis();
      if(PRINT) Serial.print("\nHeading start: ");
      if(PRINT) Serial.print(heading_start);
      if(PRINT) Serial.print("\n");
    }
    go_state = 0;
    locked_dir = false;
  }

  if(left_counter >= 5) {                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     
    left_counter = 0;
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
    if(PRINT) Serial.print("S ");
    digitalWrite(superbright_l, HIGH);
    digitalWrite(superbright_r, HIGH);
    leftBork();
    motor_setDir(1, MOTOR_DIR_FWD);
    motor_setSpeed(1, the_speed);
    motor_setDir(0, MOTOR_DIR_FWD);
    motor_setSpeed(0, the_speed);
    break;
    case 1: // left
    if(PRINT) Serial.print("L ");
    digitalWrite(superbright_l, HIGH);
    digitalWrite(superbright_r, LOW);
    leftBork();
    motor_setDir(1, MOTOR_DIR_FWD);
    motor_setSpeed(1, 60);//the_speed);
    motor_setDir(0, MOTOR_DIR_FWD);
    motor_setSpeed(0, the_speed);
    break;
    case 2: // right
    if(PRINT) Serial.print("R ");
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
  if(PRINT) Serial.print("\n");
  //delay(250);
  
}


float getCurrentHeading() {

  float yaw = 0.0;

  sensors_event_t accel_event;
  sensors_event_t mag_event;
  sensors_vec_t   orientation;

  /* Read the accelerometer and magnetometer */
  accel.getEvent(&accel_event);
  mag.getEvent(&mag_event);

  if(mag_event.magnetic.x == 0.0 && mag_event.magnetic.y == 0.0 && mag_event.magnetic.z == 0.0) {
    mag_error++;
    return 999.99;
  }

  if(accel_event.acceleration.x == 0.0 && accel_event.acceleration.y == 0.0 && accel_event.acceleration.z == 0.0) {
    accel_error++;
    return 999.99;
  }

  /* Use the new fusionGetOrientation function to merge accel/mag data */  
  if (dof.fusionGetOrientation(&accel_event, &mag_event, &orientation))
  {
    /* 'orientation' should have valid .roll and .pitch fields */
    //Serial.print(F("Orientation: "));
    //Serial.print(orientation.roll);
    //Serial.print(F(" "));
    //Serial.print(orientation.pitch);
    //Serial.print(F(" "));

    yaw = orientation.heading;
    yaw -= 45; // set 0 to north
    if(yaw < 0) yaw += 360.0;
    //Serial.println(yaw);
    //delay(100);
  }

  return yaw;
  
}





float getCurrentHeading_OLD() {
  sensors_event_t event_accl;
  sensors_event_t event_magn;

  accel.getEvent(&event_accl);
  mag.getEvent(&event_magn);

  if(event_magn.magnetic.x == 0.0 && event_magn.magnetic.y == 0.0 && event_magn.magnetic.z == 0.0) {
    mag_error++;
    return 999.99;
  }

  if(event_accl.acceleration.x == 0.0 && event_accl.acceleration.y == 0.0 && event_accl.acceleration.z == 0.0) {
    accel_error++;
    return 999.99;
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


  float roll = 0.0;
  float pitch = 0.0;
  float yaw = 0.0;


  // Freescale solution
  roll = atan2(accl_y, accl_z);
  pitch = atan(-accl_x / (accl_y * sin(roll) + accl_z * cos(roll)));
   
  float magn_fy_fs = magn_z * sin(roll) - magn_y*cos(roll);
  float magn_fx_fs = magn_x * cos(pitch) + magn_y * sin(pitch) * sin(roll) + magn_z * sin(pitch) * cos(roll);
   
  yaw = atan2(magn_fy_fs, magn_fx_fs);
   
  roll = roll * (180/PI);
  pitch = pitch * (180/PI);
  yaw = yaw * (180/PI);

  // code from adafruit 10DOF library
  // Adafruit_10DOF::magGetOrientation with SENSOR_AXIS_Z
  float heading = (float)atan2(magn_y, magn_x) * 180 / PI;

  if (heading < 0) {
    heading = 360 + heading;
  }

  if(yaw < 0) yaw = 360 + yaw;

  /*
  Serial.print(" Yaw: ");
  Serial.print(yaw);
  Serial.print(" Prev:");
  Serial.print(heading);
  Serial.print("\n");
  */
  
  return yaw;
}



