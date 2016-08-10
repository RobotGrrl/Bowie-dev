#include <Streaming.h>

int sensorA = A10;
int sensorB = A11;

void setup() {
  Serial.begin(19200);
  pinMode(sensorA, INPUT);
  pinMode(sensorB, INPUT);
}

void loop() {
  Serial << "A: " << analogRead(sensorA) << " B: " << analogRead(sensorB) << endl;
  delay(100);
}


