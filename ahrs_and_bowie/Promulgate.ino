
void received_action(char action, char cmd, uint8_t key, uint16_t val, char delim) {

  /* API
   * ----
   * L = left motor
   * R = right motor
   * S = arm
   * T = tilt / center of gravity
   * Z = soil sample leds
   * C = claw
   * H = set new target heading
   * G = follow target heading
   * P = stop following target heading
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

    if(cmd == 'L' && AUTONOMOUS == false) { // left motor
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
  
    if(cmd == 'R' && AUTONOMOUS == false) { // right motor
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

  if(action == '@') {

    if(cmd == 'L' && AUTONOMOUS == false) { // left motor
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
  
    if(cmd == 'R' && AUTONOMOUS == false) { // right motor
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

  if(cmd == 'H') { // set new compass target heading
    Serial << "prev target heading: " << target;
    target = heading_result;
    Serial << " new target heading: " << target << endl;
  }

  if(cmd == 'G') { // follow compass heading
    //Serial << 
    FOLLOW_HEADING = true;
    AUTONOMOUS = true;
  }

  if(cmd == 'P') { // stop following compass heading
    FOLLOW_HEADING = false;
    AUTONOMOUS = false;
  }
   
}

void transmit_complete() {
  
}

