void nunchuckDebug() {
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
}


void buttonTest() {
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

  //if(redbouncer.read() == HIGH) digitalWrite(LED1, HIGH);

  if(redbouncer.fell()) {
    digitalWrite(LED1, LOW);
    red_on = !red_on;
  }

  if(red_on) {
    digitalWrite(LED1, HIGH);
  }

  
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

