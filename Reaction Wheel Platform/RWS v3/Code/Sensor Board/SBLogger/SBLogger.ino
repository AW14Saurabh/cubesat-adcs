// Program for Sensor Board v2
// Gathers data from sensors + logs to uSD card
// Sensors:
// - L3GD20H (Gyro)
// - BMP280 (Temp/Pressure)
// - ADXL345 (Accel)
// - QMC5883 (Compass)
// - DS3231 (Real Time Clock)
// Author: Mark Yeo
// Last Modified 2018-04-23

// Sensors are polled on average every 26-32ms. Max poll delay <70ms (v. occasional)
// Green LED:   Data is being logged to SD card
// Yellow LED:  Data is not being logged to SD card
// Data is in format:
//      "YYYY-MM-DD HH:MM:SS MILLIS DT g: GYROX, GYROY, GYROZ a:AXLX, AXLY, AXLZ m:MAGX,MAGY,MAGZ "
// e.g. "2018-04-23 01:36:05 172670 66 g:0.0168,0.1425,0.0058 a:0.08,-0.24,11.65 m:-770,2747,5525 "



//bugs:
// gyro measurements sometimes -0.0002,-0.0002,-0.0002 + messes up tail end of mag data



#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_ADXL345_U.h>
#include <MechaQMC5883.h>
#include <RTClib.h>
#include <SPI.h>
#include <SD.h>


#define LED_G 6
#define LED_Y 9
#define PB    5
#define RTC_CLK 17
#define SD_CS 14
#define FILE_NAME "data.txt"




DS3231 rtc; //if garbage values (e.g. "20@5-25-@5  @5:@5:85"), make sure Wire.begin(); is run + no other devices at 0x68
Adafruit_L3GD20_Unified gyro = Adafruit_L3GD20_Unified(20); //assign unique ID
Adafruit_BMP280 bmp;
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
MechaQMC5883 qmc;
File dataFile;


void setup() {
  //=================================
  // INITIALISATION
  //=================================
  pinMode(LED_G, OUTPUT);
  pinMode(LED_Y, OUTPUT);
  pinMode(SD_CS, OUTPUT);
  digitalWrite(LED_G, LOW);
  digitalWrite(LED_Y, HIGH);
  //digitalWrite(SD_CS, HIGH);
  pinMode(PB, INPUT);
  pinMode(RTC_CLK, INPUT);

  //Serial.begin(9600);
  Wire.begin();

  rtc.begin();  // Initialise DS3231  (Real Time Clock)
  //rtc.adjust(DateTime(__DATE__, __TIME__)); //use this to set RTC time //as of 2018-04-22 RTC clock is 4s slow
  if (!rtc.isrunning()) { //if the coin battery has run out, then reset the clock to midnight 1st Jan 2000
    rtc.adjust(DateTime(0, 1, 1, 0, 0, 0));
  }
  gyro.begin(); // Initialise L3GD20H (Gyro)
  gyro.enableAutoRange(true);
  bmp.begin();  // Initialise BMP280  (Temp/Pressure) //addr 0x76 - may have to modify library addr
  accel.begin();// Initialise ADXL345 (Accel)
  accel.setRange(ADXL345_RANGE_16_G);
  accel.setDataRate(ADXL345_DATARATE_100_HZ);
  qmc.init();   // Initialise QMC5883 (Compass)
  qmc.setMode(Mode_Continuous,ODR_100Hz,RNG_2G,OSR_256);
  SD.begin(SD_CS);



  
  //=================================
  // STARTUP ROUTINE
  //=================================
  DateTime now = rtc.now();
  char buf[25];
  strncpy(buf,"YYYY-MM-DD hh:mm:ss\0",25);
  dataFile = SD.open(FILE_NAME, FILE_WRITE);
  if(dataFile){
    dataFile.println();
    dataFile.println("Sensor Board v2 Startup ");
    dataFile.print(now.format(buf));
    dataFile.print(" ");
    dataFile.println(millis());
    dataFile.close();
  } else {
    return;
  }
  
  digitalWrite(LED_Y, LOW);
  digitalWrite(LED_G, HIGH);
}



unsigned long previousMillis = 0;
int maxDt = 0;


void loop() {
  //=================================
  // POLL SENSORS
  //=================================
  DateTime now = rtc.now();
  unsigned long currentMillis = millis();
  float temp = bmp.readTemperature();
  float pa = bmp.readPressure();
  float ele = bmp.readAltitude(1021.7); //ave. sea level pressure in Sydney
  sensors_event_t eventG; 
  gyro.getEvent(&eventG);
  float wx = eventG.gyro.x;
  float wy = eventG.gyro.y;
  float wz = eventG.gyro.z;
  sensors_event_t eventA; 
  accel.getEvent(&eventA);
  float ax = eventA.acceleration.x;
  float ay = eventA.acceleration.y;
  float az = eventA.acceleration.z;
  uint16_t mx,my,mz;
  qmc.read(&mx,&my,&mz);



  
  //=================================
  // PROCESS DATA
  //=================================
  int dt = currentMillis - previousMillis;
  previousMillis = currentMillis;
  if (dt > maxDt && dt < 100){
    maxDt = dt;
  }


  //=================================
  // DATA FORMATTING
  //=================================
  char buf[25];
  strncpy(buf,"YYYY-MM-DD hh:mm:ss \0",25);
  char strT[25];
  sprintf(strT, "%lu %d ", currentMillis, maxDt);
  String strW = "g:" + String(wx,4) + "," + String(wy,4) + "," + String(wz,4) + " ";
  String strTP = "t:" + String(temp,2) + " p:" + String(pa,2) + " e:" + String(ele,2) + " ";
  String strA = "a:" + String(ax,2) + "," + String(ay,2) + "," + String(az,2) + " ";
  char strM[20];
  sprintf(strM, "m:%4d,%4d,%4d ", mx, my, mz);


  //=================================
  // DATA OUTPUT
  //=================================

  //Output to Serial when button is pressed (arduino sometimes bugs out w/ uploading when Serial is constantly in use)
  /*
  if (digitalRead(PB) == HIGH){
    Serial.print(now.format(buf));
    Serial.print(strT);
    Serial.print(strW);
    Serial.print(strTP);
    Serial.print(strA);
    Serial.print(strM);
    Serial.println();
  }
  */
  //Write to SD card
  dataFile = SD.open(FILE_NAME, FILE_WRITE);
  if(dataFile){
    dataFile.print(now.format(buf));
    dataFile.print(strT);
    dataFile.print(strW);
    dataFile.print(strTP);
    dataFile.print(strA);
    dataFile.print(strM);
    dataFile.println();
    dataFile.close();
    digitalWrite(LED_G, HIGH);
    digitalWrite(LED_Y, LOW);
  } else {
    digitalWrite(LED_G, LOW);
    digitalWrite(LED_Y, HIGH);
  }
}

