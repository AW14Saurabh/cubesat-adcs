// Program for debugging Sensor Board v1
// Gathers time-related data from sensors + logs to uSD card
// Sensors used:
// - DS3231 (Real Time Clock)
// Author: Mark Yeo
// Last Modified 2017-09-12


#include <Wire.h>
#include <FaBo9Axis_MPU9250.h>
#include <I2Cdev.h>
#include <RTClib.h>
#include <MPU6050.h>
#include <SD.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

#define address 0x1E //0011110b, I2C 7bit address of HMC5883
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
FaBo9Axis fabo_9axis;
MPU6050 accelgyro(0x69);
Adafruit_BMP280 bmp;
//unsigned long pwrRstTimer;
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
  // Initialise MPU9 + BMP280
  //=================================
  digitalWrite(MPU6_CS, LOW);
  fabo_9axis.begin();
  fabo_9axis.setMPU(0x69); //set address of MPU9
  fabo_9axis.configMPU9250(MPU9250_GFS_2000, MPU9250_AFS_16G);
    //MPU9250_GFS_250
    //MPU9250_GFS_500
    //MPU9250_GFS_1000
    //MPU9250_GFS_2000 <-- ?
    //
    //MPU9250_AFS_2G
    //MPU9250_AFS_4G
    //MPU9250_AFS_8G
    //MPU9250_AFS_16G <-- flight is 5g, reentry is 20g, better low res than clip values
  bmp.begin();
  digitalWrite(MPU6_CS, HIGH);
  



  //=================================
  // Initialise MPU6
  //=================================
  
  digitalWrite(MPU9_CS, LOW);
  accelgyro.initialize();
  //Set Full-Scales
  accelgyro.setFullScaleGyroRange(MPU6050_GYRO_FS_500);
  accelgyro.setFullScaleAccelRange(MPU6050_ACCEL_FS_16);
    //MPU6050_GYRO_FS_250   +-250deg/s
    //MPU6050_GYRO_FS_500   +-500deg/s
    //MPU6050_GYRO_FS_1000  +-1000deg/s
    //MPU6050_GYRO_FS_2000  +-2000deg/s <-- ?
    //
    //MPU6050_ACCEL_FS_2    +-2g
    //MPU6050_ACCEL_FS_4    +-4g
    //MPU6050_ACCEL_FS_8    +-8g
    //MPU6050_ACCEL_FS_16   +-16g <-- flight is 5g, reentry is 20g, better low res than clip values
  digitalWrite(MPU9_CS,HIGH);


  //=================================
  // Initialise HMC5883L
  //=================================
  //Put the HMC5883 IC into the correct operating mode
  Wire.beginTransmission(address); //open communication with HMC5883
  Wire.write(0x02); //select mode register
  Wire.write(0x00); //continuous measurement mode
  Wire.endTransmission();

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
  if(SD.begin(SD_CS)){ //correct?
  } else{
    return;
  }
  digitalWrite(RED_LED, LOW);
  digitalWrite(GRN_LED, HIGH);
}





void loop() {
  //=================================
  // Poll MPU9250
  //=================================
  float ax,ay,az;
  float gx,gy,gz;
  float mx,my,mz;
  float temp;
  float temp1, pa, ele;

  //Poll sensor
  digitalWrite(MPU6_CS, LOW);
  fabo_9axis.readAccelXYZ(&ax,&ay,&az);
  fabo_9axis.readGyroXYZ(&gx,&gy,&gz);
  fabo_9axis.readMagnetXYZ(&mx,&my,&mz);
  fabo_9axis.readTemperature(&temp);
  temp1 = bmp.readTemperature();
  pa = bmp.readPressure();
  ele = bmp.readAltitude(1013.25); //sea level pressure
  digitalWrite(MPU6_CS, HIGH);


  //=================================
  // Poll MPU6050
  //=================================
  int16_t ax1,ay1,az1;
  int16_t gx1,gy1,gz1;

  //Poll sensor
  digitalWrite(MPU9_CS,LOW);
  accelgyro.getMotion6(&ax1, &ay1, &az1, &gx1, &gy1, &gz1);
  digitalWrite(MPU9_CS, HIGH);


  //=================================
  // Poll HMC5883L
  //=================================
  int mx1,my1,mz1;
  
  //Poll Sensor
  //Tell the HMC5883L where to begin reading data
  Wire.beginTransmission(address);
  Wire.write(0x03); //select register 3, X MSB register
  Wire.endTransmission();
  //Read data from each axis, 2 registers per axis
  Wire.requestFrom(address, 6);
  if (6 <=Wire.available()){
     mx1 = Wire.read()<<8; //X msb
     mx1 |= Wire.read(); //X lsb
     mz1 = Wire.read()<<8; //Z msb
     mz1 |= Wire.read(); //Z lsb
     my1 = Wire.read()<<8; //Y msb
     my1 |= Wire.read(); //Y lsb
  }
  

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

  //Format sensor data //too big
  //---MPU9 data---
  //String mpu9accel = "a:" + String(ax,6) + ":" + String(ay,6) + ":" + String(az,6);
  //String mpu9gyro = "g:" + String(gx,3) + ":" + String(gy,3) + ":" + String(gz,3);
  //String mpu9mag = "m:" + String(mx,3) + ":" + String(my,3) + ":" + String(mz,3);
  //String mpu9temp = "t:" + String(temp,3);
  //---BMP data---
  //String bmptpe = "t1:" + String(temp1,3) + " p:" + String(pa,3) + " e:" + String(ele,3);
  //---MPU6 data---
  //String mpu6accel = "a1:" + String(ax1,DEC) + ":" + String(ay1,DEC) + ":" + String(az1,DEC);
  //String mpu6gyro = "g1:" + String(gx1,DEC) + ":" + String(gy1,DEC) + ":" + String(gz1,DEC);
  //---HMC5 data---
  //String hmc5mag= "m1:" + String(mx1) + ":" + String(my1) + ":" + String(mz1);
  //---Timestamp---
  char buf[100];
  strncpy(buf,"DD.MM.YYYY  hh:mm:ss\0",100);
  //String rtctime = now.format(buf);
  //String mstime = "ms:" + String(mselapsed) + "\n";

  //Output to Serial when button is pressed
  //if (digitalRead(PSH_BTN) == HIGH){ //arduino doesn't like to upload when Serial is constantly in use
    //Serial.println(mpu9accel);
    //Serial.println(mpu9gyro);
    //Serial.println(mpu9mag);
    //Serial.println(mpu9temp);
    //Serial.println(bmptpe);
    //Serial.println(mpu6accel);
    //Serial.println(mpu6gyro);
    //Serial.println(hmc5mag);  
    //Serial.println(rtctime);
    //Serial.println(mstime);
    //Serial.println();
 
  //}



  
  
  //Write to SD card
  dataFile = SD.open(FILE_NAME, FILE_WRITE);
  if(dataFile){

    dataFile.println(ax,6);
    dataFile.println(ay,6);
    dataFile.println(az,6);
    dataFile.println(gx,3);
    dataFile.println(gy,3);
    dataFile.println(gz,3);
    dataFile.println(mx,3);
    dataFile.println(my,3);
    dataFile.println(mz,3);
    dataFile.println(temp,3);
    dataFile.println(temp1,3);
    dataFile.println(pa,3);
    dataFile.println(ele,3);
    dataFile.println(ax1);
    dataFile.println(ay1);
    dataFile.println(az1);
    dataFile.println(gx1);
    dataFile.println(gy1);
    dataFile.println(gz1);
    dataFile.println(mx1);
    dataFile.println(my1);
    dataFile.println(mz1);
    dataFile.println(now.format(buf));
    dataFile.println(mselapsed);
    //dataFile.println((unsigned long) 120000 - (millis() - pwrRstTimer));
    if (printReset){
      dataFile.println("Power boards reset");
      printReset = false;
    }
    if (printStart){
      dataFile.println("Power boards startup");
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

