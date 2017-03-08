
void compassAHRS() {

  sensors_event_t gyro_event;
  sensors_event_t accel_event;
  sensors_event_t mag_event;

  // Get new data samples
  gyro.getEvent(&gyro_event);
  accel.getEvent(&accel_event);
  mag.getEvent(&mag_event);

  // Apply mag offset compensation (base values in uTesla)
  float x = mag_event.magnetic.x - mag_offsets[0];
  float y = mag_event.magnetic.y - mag_offsets[1];
  float z = mag_event.magnetic.z - mag_offsets[2];

  // Apply mag soft iron error compensation
  float mx = x * mag_softiron_matrix[0][0] + y * mag_softiron_matrix[0][1] + z * mag_softiron_matrix[0][2];
  float my = x * mag_softiron_matrix[1][0] + y * mag_softiron_matrix[1][1] + z * mag_softiron_matrix[1][2];
  float mz = x * mag_softiron_matrix[2][0] + y * mag_softiron_matrix[2][1] + z * mag_softiron_matrix[2][2];

  // The filter library expects gyro data in degrees/s, but adafruit sensor
  // uses rad/s so we need to convert them first (or adapt the filter lib
  // where they are being converted)
  float gx = gyro_event.gyro.x * 57.2958F;
  float gy = gyro_event.gyro.y * 57.2958F;
  float gz = gyro_event.gyro.z * 57.2958F;

  // Update the filter
  filter.update(gx, gy, gz,
                accel_event.acceleration.x, accel_event.acceleration.y, accel_event.acceleration.z,
                mx, my, mz);

  // Print the orientation filter output
  float roll = filter.getRoll();
  float pitch = filter.getPitch();
  float heading = filter.getYaw();


  // ------- filter it

  heading_reading[1] = heading_reading[0];
  heading_reading[0] = heading;

  if(num_sample < READ_SAMPLES) {

    heading_total += PREV_WEIGHT * heading_reading[1] + (1-PREV_WEIGHT) * heading_reading[0];
    
    num_sample++;
  } else {

    heading_result = heading_total / (READ_SAMPLES);

    // Serial.print(millis());
    // Serial.print("\t\t");
    // Serial.print(heading_result);
    // Serial.print("\n");

    heading_total = 0;
    num_sample = 0;
  }

  // --------

  // ------- go to a target

  if(millis() > 5000) { // ensure the program has been running for a bit

    int the_speed = 255;

    float opp_angle;
    float current = heading_result;

    alpha = target - current;
    if(alpha > 180) alpha -= 360;
    if(alpha < -180)  alpha += 360;

    if(alpha < 0) {
      clockwise = true;
    } else {
      clockwise = false;
    }
    
    Serial.print(millis());
    Serial.print("\tH: ");
    Serial.print(heading_result);
    Serial.print("\tA: ");
    Serial.print(alpha);
    Serial.print("\t");

    boolean motors_enabled = false;

    if(motors_enabled) {
  
      if(abs(alpha)<30.0) {
          
        digitalWrite(superbright_l, HIGH);
        digitalWrite(superbright_r, LOW);
        leftBork();
        motor_setDir(1, MOTOR_DIR_FWD);
        motor_setSpeed(1, 255);
        motor_setDir(0, MOTOR_DIR_FWD);
        motor_setSpeed(0, 255);
  
      } else {
  
        if(clockwise) {
          Serial.print("CW\t");
    
          digitalWrite(superbright_l, LOW);
          digitalWrite(superbright_r, HIGH);
          leftBork();
          motor_setDir(0, MOTOR_DIR_REV);
          motor_setSpeed(0, 150);
          motor_setDir(1, MOTOR_DIR_FWD);
          motor_setSpeed(1, the_speed);
          
          
        } else {
          Serial.print("CCW\t");
    
          digitalWrite(superbright_l, HIGH);
          digitalWrite(superbright_r, LOW);
          leftBork();
          motor_setDir(1, MOTOR_DIR_REV);
          motor_setSpeed(1, 150);
          motor_setDir(0, MOTOR_DIR_FWD);
          motor_setSpeed(0, the_speed);
          
        }
  
      }

    }

    Serial.print("\n");

  }
  
  // --------

}


