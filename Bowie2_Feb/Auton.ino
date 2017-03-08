void distanceSensors() {

  float left = analogRead(SONAR_LEFT);
  delay(20);
  float right = analogRead(SONAR_RIGHT);

  float left_dist = ( (float)left * (3.3/512.0) ) * 2.54;
  float right_dist = ( (float)right * (3.3/512.0) ) * 2.54;

//    if(millis()-last_print >= 100) {
//      Serial1 << "L: " << left_dist << "cm R: " << right_dist << "cm" << endl;
//      last_print = millis();
//    }


  // count the trigs
  if(left_dist < dist_thresh && right_dist < dist_thresh) {
    if(millis()-last_print >= 100) {
      Serial1 << "SS " << "Both triggered! L: " << left_dist << "cm R: " << right_dist << endl;
      last_print = current_time;
    }
    
    both_trig++;
    last_both_trig = current_time;
    
  } else if(left_dist < dist_thresh) {
    if(millis()-last_print >= 100) {
      Serial1 << "SS " << "Left triggered! " << left_dist << "cm" << endl;
      last_print = current_time;
    }

    left_trig++;
    last_left_trig = current_time;
    
  } else if(right_dist < dist_thresh) {
    if(millis()-last_print >= 100) {
      Serial1 << "SS " << "Right triggered! " << right_dist << "cm" << endl;
      last_print = current_time;
    }

    right_trig++;
    last_right_trig = current_time;
    
  }


  // clear out the goopy trigs
  if(current_time-last_both_trig >= 1000) {
    both_trig = 0;
  }

  if(current_time-last_left_trig >= 1000) {
    left_trig = 0;
  }

  if(current_time-last_right_trig >= 1000) {
    right_trig = 0;
  }


  result_n2 = result_n1;
  result_n1 = result;

  // determine what to do based on trigs
  if(both_trig >= 10) {
    // backup 

    // just keep backing up regardless of n-1, n-2
    
    backward(2000);
    result = 3;
    
  } else if(left_trig >= 10) {
    // go right

    backward(500);
    rightEasy(1000);
    result = 2;

    /*
    if(result_n1 == 1) {
      // backup
    } else if(result_n1 == 2) {
      // 
    } else if(result_n1 == 3) {
      // 
    }
    */
    
  } else if(right_trig >= 10) {
    // go left

    backward(500);
    leftEasy(1000);
    result = 1;
    
  }

  
}















void sweepMode() {
    
  int claw_down = 1500;
  int claw_up = 1100;
  int arm_down = 180;
  int arm_up = 150;
  
  boolean fast_mode = true;
  
  arm.write(180-arm_up);
  arm2.write(arm_up);
  claw.writeMicroseconds(claw_up);
  
  
  if(!fast_mode) delay(1000);
  
  
  while(1<3) {
  
    // down
    for(int i=arm_up; i<arm_down; i++) {
      arm.write(180-i);
      arm2.write(i);
      delay(15);
    }
  
    for(int i=claw_up; i<claw_down; i++) {
      claw.writeMicroseconds(i);  
      delay(1);
    }
  
  
    // backward
    digitalWrite(superbright_l, LOW);
    leftBork();
    motor_setDir(0, MOTOR_DIR_REV);
    motor_setSpeed(0, 255);
  
    digitalWrite(superbright_r, LOW);
    motor_setDir(1, MOTOR_DIR_REV);
    motor_setSpeed(1, 255);
  
  
    delay(2000);
  
  
    // stop
    digitalWrite(superbright_l, LOW);
    leftBork();
    motor_setDir(0, MOTOR_DIR_REV);
    motor_setSpeed(0, 0);
  
    digitalWrite(superbright_r, LOW);
    motor_setDir(1, MOTOR_DIR_REV);
    motor_setSpeed(1, 0);
  
  
    if(!fast_mode) delay(2000);
  
  
    // up
    for(int i=arm_down; i>arm_up; i--) {
      arm.write(180-i);
      arm2.write(i);
      delay(1);
    }
    
    for(int i=claw_down; i>claw_up; i--) {
      claw.writeMicroseconds(i);  
      delay(1);
    }
    
    
    // forward
    digitalWrite(superbright_l, HIGH);
    leftBork();
    motor_setDir(0, MOTOR_DIR_FWD);
    motor_setSpeed(0, 255);
  
    digitalWrite(superbright_r, HIGH);
    motor_setDir(1, MOTOR_DIR_FWD);
    motor_setSpeed(1, 255);  
  
  
    delay(1500);
  
  
    // turn
    digitalWrite(superbright_l, LOW);
    leftBork();
    motor_setDir(0, MOTOR_DIR_REV);
    motor_setSpeed(0, 255);
  
    digitalWrite(superbright_r, HIGH);
    motor_setDir(1, MOTOR_DIR_FWD);
    motor_setSpeed(1, 255);  
  
  
    delay(200);
  
  
    // stop
    digitalWrite(superbright_l, LOW);
    leftBork();
    motor_setDir(0, MOTOR_DIR_REV);
    motor_setSpeed(0, 0);
  
    digitalWrite(superbright_r, LOW);
    motor_setDir(1, MOTOR_DIR_REV);
    motor_setSpeed(1, 0);
    
    
  }

}


void stopGoing() {
  digitalWrite(superbright_l, LOW);
    leftBork();
    motor_setDir(0, MOTOR_DIR_REV);
    motor_setSpeed(0, 0);
  
    digitalWrite(superbright_r, LOW);
    motor_setDir(1, MOTOR_DIR_REV);
    motor_setSpeed(1, 0);
}

void theWave() {
  /*
  for(int i=0; i<3; i++) {
    claw.writeMicroseconds(claw_min);
    delay(100);

      for(int i=arm_up; i<arm_down; i++) {
        arm.write(180-i);
        arm2.write(i);
        delay(1);
      }
    
    claw.writeMicroseconds(claw_max);
    delay(100);

      for(int i=arm_down; i>arm_up; i--) {
        arm.write(180-i);
        arm2.write(i);
        delay(1);
      }
    
  }
  */
}


void autoDig() {
  
  int claw_down = 1200;
  int claw_up = 900;
  int arm_down = 180;
  int arm_up = 150;
  
  boolean fast_mode = true;

  arm.write(180-arm_up);
  arm2.write(arm_up);
  claw.writeMicroseconds(claw_up);
  
  if(!fast_mode) delay(1000);

while(1<3) {

  // go to some place!
  digitalWrite(superbright_l, HIGH);
  leftBork();
  motor_setDir(0, MOTOR_DIR_FWD);
  motor_setSpeed(0, 255);
  
  digitalWrite(superbright_r, HIGH);
  motor_setDir(1, MOTOR_DIR_FWD);
  motor_setSpeed(1, 255);  
  
  delay(2000);
  stopGoing();

  if(!fast_mode) delay(2000);
  

  // move arm down
  for(int i=130; i<arm_down; i++) {
    arm.write(180-i);
    arm2.write(i);
    delay(15);
  }

  if(!fast_mode) delay(2000);

  // move claw down
  for(int i=claw_up; i<claw_down; i++) {
    claw.writeMicroseconds(i);  
    delay(1);
  }

  if(!fast_mode) delay(2000);

  
  // go forward
  digitalWrite(superbright_l, HIGH);
  leftBork();
  motor_setDir(0, MOTOR_DIR_FWD);
  motor_setSpeed(0, 255);

  digitalWrite(superbright_r, HIGH);
  motor_setDir(1, MOTOR_DIR_FWD);
  motor_setSpeed(1, 255);  

  delay(1000);
  stopGoing();

  if(!fast_mode) delay(2000);

  // turn a little (quick)
  digitalWrite(superbright_l, LOW);
  leftBork();
  motor_setDir(0, MOTOR_DIR_REV);
  motor_setSpeed(0, 255);

  digitalWrite(superbright_r, HIGH);
  motor_setDir(1, MOTOR_DIR_FWD);
  motor_setSpeed(1, 255);  

  delay(200);
  stopGoing();

  if(!fast_mode) delay(2000);

  digitalWrite(superbright_l, LOW);
  leftBork();
  motor_setDir(0, MOTOR_DIR_FWD);
  motor_setSpeed(0, 255);

  digitalWrite(superbright_r, HIGH);
  motor_setDir(1, MOTOR_DIR_REV);
  motor_setSpeed(1, 255);  

  delay(200);
  stopGoing();

  if(!fast_mode) delay(2000);

  // go backward
  digitalWrite(superbright_l, LOW);
  leftBork();
  motor_setDir(0, MOTOR_DIR_REV);
  motor_setSpeed(0, 255);

  digitalWrite(superbright_r, LOW);
  motor_setDir(1, MOTOR_DIR_REV);
  motor_setSpeed(1, 255);

  delay(200);
  stopGoing();
  
  if(!fast_mode) delay(2000);
  
  
  // claw up
  for(int i=claw_down; i>claw_up; i--) {
    claw.writeMicroseconds(i);  
    delay(1);
  }

  if(!fast_mode) delay(2000);

  // arm up
  for(int i=arm_down; i>130; i--) {
    arm.write(180-i);
    arm2.write(i);
    delay(15);
  }

  if(!fast_mode) delay(2000);


    // turn
  digitalWrite(superbright_l, LOW);
  leftBork();
  motor_setDir(0, MOTOR_DIR_REV);
  motor_setSpeed(0, 255);

  digitalWrite(superbright_r, HIGH);
  motor_setDir(1, MOTOR_DIR_FWD);
  motor_setSpeed(1, 255);  

  delay(1000);
  stopGoing();

  if(!fast_mode) delay(2000);


  // claw down

  for(int i=130; i<150; i++) {
    arm.write(180-i);
    arm2.write(i);
    delay(5);
  }
  
  for(int j=0; j<3; j++) {

    for(int i=claw_up; i<1700; i++) {
      claw.writeMicroseconds(i);  
      //delay(1);//Microseconds(10);
    }
    delay(250);

    for(int i=1700; i>claw_up; i--) {
      claw.writeMicroseconds(i);  
      //delay(1);//Microseconds(10);
    }
    delay(100);

  }

  for(int i=150; i<160; i++) {
    arm.write(180-i);
    arm2.write(i);
    delay(15);
  }

  if(!fast_mode) delay(2000);



}


}


void traverseAreaMode() {
    
  boolean fast_mode = true;
  int spd = 220;
  
  if(!fast_mode) delay(1000);
  
  
  while(1<3) {
  
    // forward
    digitalWrite(superbright_l, HIGH);
    leftBork();
    motor_setDir(0, MOTOR_DIR_FWD);
    motor_setSpeed(0, spd);
  
    digitalWrite(superbright_r, HIGH);
    motor_setDir(1, MOTOR_DIR_FWD);
    motor_setSpeed(1, spd);  
  
  
    delay(3000);
  

    for(int i=0; i<5; i++) {
  
    // turn
    digitalWrite(superbright_l, LOW);
    leftBork();
    motor_setDir(0, MOTOR_DIR_REV);
    motor_setSpeed(0, spd);
  
    digitalWrite(superbright_r, HIGH);
    motor_setDir(1, MOTOR_DIR_FWD);
    motor_setSpeed(1, spd);  
  
  
    delay(500);


    // bward
    digitalWrite(superbright_l, HIGH);
    leftBork();
    motor_setDir(0, MOTOR_DIR_REV);
    motor_setSpeed(0, spd);
  
    digitalWrite(superbright_r, HIGH);
    motor_setDir(1, MOTOR_DIR_REV);
    motor_setSpeed(1, spd);  
  
  
    delay(200);

    }

    
  }

}



