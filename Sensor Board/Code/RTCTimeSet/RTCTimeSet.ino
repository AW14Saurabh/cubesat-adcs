// Program for Sensor Board v1
//  Sets time on RTC chip to the compile time
//  Compilation to runtime usually takes ~5s, so RTC lags behind computer time by ~5s
//  Ensure no power interrupts between running this code and uploading full Sensor Board code
// Author: Mark Yeo
// Last Modified 2017-08-04

#include <Wire.h>
#include <I2Cdev.h>
#include <RTClib.h>


DS3231 rtc;

void setup() {
  
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

