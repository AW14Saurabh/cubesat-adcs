// Program for Sensor Board v1
//  Sets time on RTC chip to the compile time
//  Compilation to runtime usually takes ~5s, so RTC lags behind computer time by ~5s
//  Ensure no power interrupts between running this code and uploading full Sensor Board code
// Author: Mark Yeo
// Last Modified 2017-08-04

#include <Wire.h>
#include <I2Cdev.h>
#include <RTClib.h>

#define PWR_RST 4
#define RED_LED 5
#define GRN_LED 6
#define MPU9_CS 7
#define MPU6_CS 8
#define PSH_BTN 9
#define SD_CS 10


DS3231 rtc;

void setup() {
  pinMode(MPU9_CS, OUTPUT);
  pinMode(MPU6_CS, OUTPUT);
  digitalWrite(MPU9_CS, HIGH);
  digitalWrite(MPU6_CS, HIGH);
  
  //=================================
  // Join I2C bus (I2Cdev library doesn't do this automatically)
  //=================================
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif
  rtc.begin();
  rtc.adjust(DateTime(__DATE__, __TIME__));
  Serial.begin(9600);
}





void loop() {
  DateTime now = rtc.now();
  char buf[100];
  strncpy(buf,"DD.MM.YYYY  hh:mm:ss\0",100);
  Serial.println(now.format(buf));
  delay(1000);
}

