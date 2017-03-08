void backward(int del) {
  digitalWrite(superbright_l, LOW);
  leftBork();
  motor_setDir(0, MOTOR_DIR_REV);
  motor_setSpeed(0, 255);

  digitalWrite(superbright_r, LOW);
  motor_setDir(1, MOTOR_DIR_REV);
  motor_setSpeed(1, 255);

  delay(del);
}

void forward(int del) {
  digitalWrite(superbright_l, HIGH);
  leftBork();
  motor_setDir(0, MOTOR_DIR_FWD);
  motor_setSpeed(0, 255);

  digitalWrite(superbright_r, HIGH);
  motor_setDir(1, MOTOR_DIR_FWD);
  motor_setSpeed(1, 255);

  delay(del);
}

void leftEasy(int del) {
  digitalWrite(superbright_l, HIGH);
  leftBork();
  motor_setDir(0, MOTOR_DIR_FWD);
  motor_setSpeed(0, 255);

  digitalWrite(superbright_r, LOW);
  motor_setDir(1, MOTOR_DIR_FWD);
  motor_setSpeed(1, 60);

  delay(del);
}

void rightEasy(int del) {
  digitalWrite(superbright_l, LOW);
  leftBork();
  motor_setDir(0, MOTOR_DIR_FWD);
  motor_setSpeed(0, 60);

  digitalWrite(superbright_r, HIGH);
  motor_setDir(1, MOTOR_DIR_FWD);
  motor_setSpeed(1, 255);

  delay(del);
}


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

void leftBork() {
  pinMode(MOTORA_CTRL1, OUTPUT);
  digitalWrite(MOTORA_CTRL1, LOW);
  pinMode(MOTORA_CTRL2, OUTPUT);
  digitalWrite(MOTORA_CTRL2, LOW);
  pinMode(MOTORA_SPEED, OUTPUT);
  analogWrite(MOTORA_SPEED, 0);
}


