/*
 * the new an improved gps unit
 * by Erin RobotGrrl for Robot Missions
 * Aug 20, 2016
 * 
 */

#include <Wire.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <SD.h>
#include <Adafruit_MPL3115A2.h>
//#include <Adafruit_LIS3DH.h>

#define DEBUG false
#define ROBOT true

static const int RXPin = 2, TXPin = 3;
static const uint32_t GPSBaud = 9600;

TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);

File myFile;

uint8_t orange_led = 8;
uint8_t blue_led = 9;
uint8_t button = 7;
uint16_t file_num = 0;
char file_name[12];

Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2();
//Adafruit_LIS3DH lis = Adafruit_LIS3DH();


void setup() {

  pinMode(orange_led, OUTPUT);
  pinMode(blue_led, OUTPUT);
  pinMode(button, INPUT);

  for(int i=0; i<5; i++) {
    digitalWrite(orange_led, HIGH);
    digitalWrite(blue_led, LOW);
    delay(100);
    digitalWrite(orange_led, LOW);
    digitalWrite(blue_led, HIGH);
    delay(100);
  }
  digitalWrite(orange_led, LOW);
  digitalWrite(blue_led, LOW);

  
  Serial.begin(9600);
  ss.begin(GPSBaud);
  
  if (!baro.begin()) {
    if(DEBUG) Serial.println(F("Couldnt find barometer sensor"));
    return;
  }

  if (!SD.begin(10)) {
    if(DEBUG) Serial.println(F("SD initialization failed!"));
    while(1);
    return;
  }

//  if (!lis.begin(0x18)) {   // change this to 0x19 for alternative i2c address
//    if(DEBUG) Serial.println(F("Couldnt start accelerometer"));
//    return;
//  }
//  lis.setRange(LIS3DH_RANGE_4_G);
  
  

  countFileNum();
  sprintf(file_name, "LOG-%d.TXT", file_num);
  
  delay(1000);

}

void loop() {

  digitalWrite(orange_led, HIGH);
  myFile = SD.open(file_name, FILE_WRITE);

  // satellites, hdop, lat, lon, location age, date, time, date age, 
  // altitude (m), course (deg), speed (km/h), cardinal,
  // chars processed, sentences with fix, failed checksum,
  // pressure (kPa), altitude (m), temperature (centigrade),
  // acc x (m/s^2), y (m/s^2), z (m/s^2) 

  printInt(gps.satellites.value(), gps.satellites.isValid(), 5);
  printInt(gps.hdop.value(), gps.hdop.isValid(), 5);
  printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
  printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);
  printInt(gps.location.age(), gps.location.isValid(), 5);
  printDateTime(gps.date, gps.time);
  printFloat(gps.altitude.meters(), gps.altitude.isValid(), 7, 2);
  printFloat(gps.course.deg(), gps.course.isValid(), 7, 2);
  printFloat(gps.speed.kmph(), gps.speed.isValid(), 6, 2);
  printStr(gps.course.isValid() ? TinyGPSPlus::cardinal(gps.course.value()) : "*** ", 6);

  printInt(gps.charsProcessed(), true, 6);
  printInt(gps.sentencesWithFix(), true, 10);
  printInt(gps.failedChecksum(), true, 9);


  printFloat(baro.getPressure(), true, 8, 2); // kPa
  printFloat(baro.getAltitude(), true, 8, 2); // meters
  printFloat(baro.getTemperature(), true, 7, 2); // centigrade

  
//  lis.read();
//  printFloat(lis.x, true, 7, 3);
//  printFloat(lis.y, true, 7, 3);
//  printFloat(lis.z, true, 7, 3);
  
  
  if(DEBUG) Serial.println();
  myFile.println();
  myFile.close();

  if(ROBOT) {
    if(gps.location.isValid()) {
      Serial.print("A");
      Serial.print(gps.location.lat());
      Serial.print(",B");
      Serial.print(gps.location.lng());
      Serial.println();
    } else {
      Serial.print("A");
      Serial.print("44.265878");
      Serial.print(",B");
      Serial.print("-76.493912");
      Serial.println();
    }
  }
  
  smartDelay(1000);

  if (millis() > 5000 && gps.charsProcessed() < 10) {
    if(DEBUG) Serial.println(F("No GPS data received: check wiring"));
    digitalWrite(blue_led, LOW);
  } else {
    if(gps.satellites.value() > 0) {
      digitalWrite(blue_led, HIGH);
    } else {
      digitalWrite(blue_led, LOW);
    }
  }
  
  digitalWrite(orange_led, LOW);

}






// This custom version of delay() ensures that the gps object
// is being "fed".
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}

static void printFloat(float val, bool valid, int len, int prec)
{
  if (!valid)
  {
    while (len-- > 1)
      if(DEBUG) Serial.print('*');
      myFile.print('*');
    if(DEBUG) Serial.print(' ');
    myFile.print(' ');
  }
  else
  {
    if(DEBUG) Serial.print(val, prec);
    myFile.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; ++i)
      if(DEBUG) Serial.print(' ');
      myFile.print(' ');
  }
  smartDelay(0);
}

static void printInt(unsigned long val, bool valid, int len)
{
  char sz[32] = "*****************";
  if (valid)
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0) 
    sz[len-1] = ' ';
  if(DEBUG) Serial.print(sz);
  myFile.print(sz);
  smartDelay(0);
}

static void printDateTime(TinyGPSDate &d, TinyGPSTime &t)
{
  if (!d.isValid())
  {
    if(DEBUG) Serial.print(F("********** "));
    myFile.print(F("********** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
    if(DEBUG) Serial.print(sz);
    myFile.print(sz);
  }
  
  if (!t.isValid())
  {
    if(DEBUG) Serial.print(F("******** "));
    myFile.print(F("******** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
    if(DEBUG) Serial.print(sz);
    myFile.print(sz);
  }

  printInt(d.age(), d.isValid(), 5);
  smartDelay(0);
}

static void printStr(const char *str, int len)
{
  int slen = strlen(str);
  for (int i=0; i<len; ++i) {
    if(DEBUG) Serial.print(i<slen ? str[i] : ' ');
    myFile.print(i<slen ? str[i] : ' ');
  }
  smartDelay(0);
}

void countFileNum() {
  File root = SD.open("/");
   while (true) {
    
    File entry =  root.openNextFile();
    if (! entry) {
      // no more files
      break;
    }
    
    //Serial.print(entry.name());
    file_num++;
    entry.close();
  }
  root.close();
  if(DEBUG) Serial.print("Files: ");
  if(DEBUG) Serial.println(file_num);
}


