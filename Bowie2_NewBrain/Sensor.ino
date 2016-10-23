void sensorDataLog() {

  sensors_vec_t   orientation;

  // Use the simple AHRS function to get the current orientation.
  if (ahrs.getOrientation(&orientation))
  {
    Serial1 << "OR ,";
    Serial1 << "R, " << orientation.roll << ", ";
    Serial1 << "P, " << orientation.pitch << ", ";
    Serial1 << "H, " << orientation.heading << ", ";
    Serial1 << endl;
  }

  // Calculate the altitude using the barometric pressure sensor
  /*
  sensors_event_t bmp_event;
  bmp.getEvent(&bmp_event);
  if (bmp_event.pressure)
  {
    float temperature;
    bmp.getTemperature(&temperature);
    Serial1 << "BP ,";
    Serial1 << "T , " << bmp.getTemperature(&temperature);
    Serial1 << "A , " << bmp.pressureToAltitude(seaLevelPressure,
                                        bmp_event.pressure,
                                        temperature);
    Serial1 << endl;
  }
  */

  sensors_event_t accel_event;
  sensors_event_t mag_event;

  accel.getEvent(&accel_event);
  Serial1 << "AC ,";
  Serial1 << "X," << accel_event.acceleration.x;
  Serial1 << "Y," << accel_event.acceleration.y;
  Serial1 << "Z," << accel_event.acceleration.z;
  Serial1 << endl;

  mag.getEvent(&mag_event);
  Serial1 << "MG ,";
  Serial1 << "X," << mag_event.magnetic.x;
  Serial1 << "Y," << mag_event.magnetic.y;
  Serial1 << "Z," << mag_event.magnetic.z;
  Serial1 << endl;

}

