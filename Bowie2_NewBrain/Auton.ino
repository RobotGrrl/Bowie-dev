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

