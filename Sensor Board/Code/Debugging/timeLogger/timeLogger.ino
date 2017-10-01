// Program for debugging Sensor Board v1
// Gathers time-related data from sensors + logs to uSD card
// Sensors used:
// - DS3231 (Real Time Clock)
// Author: Mark Yeo
// Last Modified 2017-09-30


#include <Wire.h>
#include <I2Cdev.h>
#include <RTClib.h>
#include <SD.h>
#include <SPI.h>

#define PWR_RST 4
#define RED_LED 5
#define GRN_LED 6
#define MPU9_CS 7
#define MPU6_CS 8
#define PSH_BTN 9
#define SD_CS 10
#define FILE_NAME "data.txt"
#define NORMAL 'n'
#define RESET 'o'


File dataFile;
DS3231 rtc;
int startMin = 0;
int startSec = 0;
char pwrRstStatus = NORMAL;
bool printReset = false;
bool printStart = true;

void setup() {
  pinMode(PWR_RST, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(GRN_LED, OUTPUT);
  pinMode(MPU9_CS, OUTPUT);
  pinMode(MPU6_CS, OUTPUT);
  pinMode(PSH_BTN, INPUT);
  pinMode(SD_CS, OUTPUT);
  digitalWrite(PWR_RST, HIGH);
  digitalWrite(RED_LED, HIGH);
  digitalWrite(GRN_LED, LOW);
  digitalWrite(MPU9_CS, HIGH);
  digitalWrite(MPU6_CS, HIGH);
  digitalWrite(SD_CS, HIGH);
  
  //=================================
  // Join I2C bus (I2Cdev library doesn't do this automatically)
  //=================================
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif
  Serial.begin(9600);

  //=================================
  // Initialise Clock + Reset Timer
  //=================================
  //Start the timer
  rtc.begin();
  //rtc.adjust(DateTime(__DATE__, __TIME__));
  if (!rtc.isrunning()) { //if the battery's run out, then just reset the clock to midnight 1st Jan 2000
    //rtc.adjust(DateTime(__DATE__, __TIME__));
    rtc.adjust(DateTime(0, 1, 1, 0, 0, 0));
  }
  DateTime now = rtc.now();
  startMin = now.minute();
  startSec = now.second();
  
  //=================================
  // Initialise SD card module
  //=================================
  if(SD.begin(SD_CS)){
  } else{
    return;
  }
  
  digitalWrite(RED_LED, LOW);
  digitalWrite(GRN_LED, HIGH);
  
  //---Startup Timestamp---
  char buf[100];
  strncpy(buf,"DD.MM.YYYY  hh:mm:ss\0",100);
  
  //Write to SD card
  dataFile = SD.open(FILE_NAME, FILE_WRITE);
  if(dataFile){
    dataFile.print("SBStartup ");
    dataFile.print(now.format(buf));
    dataFile.print(" ");
    dataFile.println(millis());
    dataFile.println();
    dataFile.close();
  }
}





void loop() {

  //=================================
  // Poll RTC + Inbuilt Clock
  //=================================
  DateTime now = rtc.now();
  unsigned long mselapsed = millis();




  //=================================
  // RESET POWER BOARDS
  //=================================

//  if (pwrRstStatus == NORMAL && (millis() - pwrRstTimer > (unsigned long) 120000)){
  if (pwrRstStatus == NORMAL && (now.minute() - startMin) % 5 == 0 && now.second() - startSec == 0){
    digitalWrite(PWR_RST, LOW);
    digitalWrite(RED_LED, HIGH);
    pwrRstStatus = RESET;
    printReset = true;
  }
  //Serial.println((unsigned long) (5*60000) - (millis() - pwrRstTimer));
//  if (pwrRstStatus == RESET && (millis() - pwrRstTimer > (unsigned long) 121000)){
  if (pwrRstStatus == RESET && now.second() - startSec >= 1){
    digitalWrite(PWR_RST, HIGH);
    digitalWrite(RED_LED, LOW);
    pwrRstStatus = NORMAL;
    printStart = true;
  }



  //=================================
  // DATA OUTPUT
  //=================================
  
  //---Timestamp---
  char buf[100];
  strncpy(buf,"DD.MM.YYYY  hh:mm:ss\0",100);


  
  
  //Write to SD card
  dataFile = SD.open(FILE_NAME, FILE_WRITE);
  if(dataFile){
    dataFile.println(now.format(buf));
    dataFile.println(mselapsed);
    if (printReset){
      dataFile.print("Power boards reset ");
      dataFile.print(now.format(buf));
      dataFile.print(" ");
      dataFile.println(mselapsed);
      printReset = false;
    }
    if (printStart){
      dataFile.print("Power boards startup ");
      dataFile.print(now.format(buf));
      dataFile.print(" ");
      dataFile.println(mselapsed);
      printStart = false;
    }
    dataFile.println();
    dataFile.close();
    digitalWrite(GRN_LED, HIGH);
  } else {
    digitalWrite(GRN_LED, LOW);
  }





  //delay(10);
}

