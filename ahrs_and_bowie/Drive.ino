void reverse(int motor, int val) {

  if(motor == 0) { // left

    leftBork();
    
    digitalWrite(superbright_l, LOW);
    
    motor_setDir(0, MOTOR_DIR_REV);
    motor_setSpeed(0, val);
    
  } else if(motor == 1) { // right
    digitalWrite(superbright_r, LOW);
    
    motor_setDir(1, MOTOR_DIR_REV);
    motor_setSpeed(1, val);
    
  }
  
}

void forward(int motor, int val) {

  if(motor == 0) { // left

    leftBork();
    
    digitalWrite(superbright_l, HIGH);
    motor_setDir(0, MOTOR_DIR_FWD);

    if(ULTRASONIC_MOTOR_OVERRIDE) {
      motor_setSpeed(0, floor(val*SPEED_ADJUST));
    } else {
      motor_setSpeed(0, val);
    }
    
  } else if(motor == 1) { // right
    digitalWrite(superbright_r, HIGH);
    motor_setDir(1, MOTOR_DIR_FWD);

    if(ULTRASONIC_MOTOR_OVERRIDE) {
      motor_setSpeed(1, floor(val*SPEED_ADJUST));
    } else {
      motor_setSpeed(1, val);
    }
    
  }

}


void leftBork() {
  pinMode(MOTORA_CTRL1, OUTPUT);
  digitalWrite(MOTORA_CTRL1, LOW);
  pinMode(MOTORA_CTRL2, OUTPUT);
  digitalWrite(MOTORA_CTRL2, LOW);
  pinMode(MOTORA_SPEED, OUTPUT);
  analogWrite(MOTORA_SPEED, 0);
}



