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


