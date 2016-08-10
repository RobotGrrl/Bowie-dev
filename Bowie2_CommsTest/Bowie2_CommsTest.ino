
int led = 13;
boolean blink_on = false;
long last_print = 0;

void setup() {
  Serial.begin(19200);
  Serial2.begin(9600);
  pinMode(led, OUTPUT);
  Serial.println("Hello");
}

void loop() {
  /*
  if(millis()-last_print >= 100) {
    Serial.print("~");
    if(blink_on) {
      digitalWrite(led, HIGH);
    } else {
      digitalWrite(led, LOW);
    }
    blink_on = !blink_on;
    last_print = millis();
  }
  */

  if(Serial2.available()) {
    char c = Serial2.read();
    //promulgate.organize_message(c);
    Serial.print(c);
    if(c == '!') Serial.print("\n");
  }
  
}

