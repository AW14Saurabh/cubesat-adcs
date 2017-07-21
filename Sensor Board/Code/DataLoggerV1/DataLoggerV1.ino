// Program for Sensor Board v1
// Gathers data from sensors + logs to uSD card
// Sensors:
// - MPU9250 (Accel/Gyro/Compass)
// - BMP280 (Temp/Pressure)
// - MPU6050 (Accel/Gyro)
// - HMC5883L (Compass)
// - DS3231 (Real Time Clock)
// Author: James Ryan + Mark Yeo
// Last Modified 2017-07-21

// TO DO:
// - see what gyro scale best for rocket launch
// Optional:
// - check how to convert gathered data into metric values



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
#define MPU6_CS 8
#define MPU9_CS 7
#define SD_CS 10
#define RED_LED 5
#define GRN_LED 6
#define BUTTON 9
#define FILE_NAME "data1.txt"


File dataFile;
//RTC_Millis rtc; //use if rtc bugging out - uses arduino's inbuilt clock, starts from 0 on powerup
DS3231 rtc;
FaBo9Axis fabo_9axis;
MPU6050 accelgyro(0x69);
Adafruit_BMP280 bmp;

void setup() {
  pinMode(MPU6_CS, OUTPUT);
  pinMode(MPU9_CS, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(GRN_LED, OUTPUT);
  pinMode(SD_CS, OUTPUT);
  pinMode(BUTTON, INPUT);
  digitalWrite(MPU6_CS, HIGH);
  digitalWrite(MPU9_CS, HIGH);
  digitalWrite(RED_LED, HIGH);
  digitalWrite(GRN_LED, LOW);

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
  accelgyro.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
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
  // Initialise Clock
  //=================================
  //Start the timer
  rtc.begin();
  if (!rtc.isrunning()) { //if the battery's run out, then just reset the clock to midnight 1st Jan 2000
    rtc.adjust(DateTime(0, 1, 1, 0, 0, 0));
  }

  //=================================
  // Initialise SD card module
  //=================================
  if(SD.begin()){
  } else{
    return;
  }
  /*
  dataFile = SD.open(FILE_NAME, FILE_WRITE); 
  if(dataFile){
    dataFile.println("================================");
    dataFile.close();
  }*/
  /*
  dataFile = SD.open(FILE_NAME, FILE_WRITE); 
  if(dataFile){
    DateTime now = rtc.now();
    char buf[100];
    strncpy(buf,"DD.MM.YYYY  hh:mm:ss\0",100);
    dataFile.println();
    dataFile.println("================================");
    dataFile.print("Power on at ");
    dataFile.println(now.format(buf));
    dataFile.println("--------------------------------");
    dataFile.println();
  } else {
    Serial.println("Error initially writing to file");
  }
  dataFile.close();
*/
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
  // Poll RTC
  //=================================
  DateTime now = rtc.now();


  //=================================
  // DATA OUTPUT
  //=================================

  //Format sensor data
  //---MPU9 data---
  String mpu9accel = "ax: " + String(ax,6) + " ay: " + String(ay,6) + " az: " + String(az,6);
  String mpu9gyro = "gx: " + String(gx,3) + " gy: " + String(gy,3) + " gz: " + String(gz,3);
  String mpu9mag = "mx: " + String(mx,3) + " my: " + String(my,3) + " mz: " + String(mz,3);
  String mpu9temp = "temp: " + String(temp,3);
  //---BMP data---
  String bmptpe = "temp1: " + String(temp1,3) + " pa: " + String(pa,3) + " ele: " + String(ele,3);
  //---MPU6 data---
  String mpu6accel = "ax1: " + String(ax1,DEC) + " ay1: " + String(ay1,DEC) + " az1: " + String(az1,DEC);
  String mpu6gyro = "gx1: " + String(gx1,DEC) + " gy1: " + String(gy1,DEC) + " gz1: " + String(gz1,DEC);
  //---HMC5 data---
  String hmc5mag= "mx1: " + String(mx1) + " my1: " + String(my1) + " mz1: " + String(mz1);
  //---Timestamp---
  char buf[100];
  strncpy(buf,"DD.MM.YYYY  hh:mm:ss\n\0",100);
  String rtctime = now.format(buf);

  //Output to Serial when button is pressed
  if (digitalRead(BUTTON) == HIGH){ //arduino doesn't like to upload when Serial is constantly in use
    Serial.println(mpu9accel);
    //Serial.println(mpu9gyro);
    //Serial.println(mpu9mag);
    //Serial.println(mpu9temp);
    //Serial.println(bmptpe);
    //Serial.println(mpu6accel);
    //Serial.println(mpu6gyro);
    //Serial.println(hmc5mag);  
    //Serial.println(rtctime);
    //Serial.println();
  
    /*
    Serial.println(mpu9accel);
    Serial.println(mpu9gyro);
    Serial.println(mpu9mag);
    Serial.println(mpu9temp);
    Serial.println(bmptpe);
    Serial.println(mpu6accel);
    Serial.println(mpu6gyro);
    Serial.println(hmc5mag);  
    Serial.println(rtctime);
    Serial.println();
    */
  }

  //Write to SD card
  dataFile = SD.open(FILE_NAME, FILE_WRITE);
  if(dataFile){
    dataFile.println(mpu9accel);
    dataFile.println(mpu9gyro);
    dataFile.println(mpu9mag);
    dataFile.println(mpu9temp);
    dataFile.println(bmptpe);
    dataFile.println(mpu6accel);
    dataFile.println(mpu6gyro);
    dataFile.println(hmc5mag);
    dataFile.println(rtctime);
    
    dataFile.close();
    digitalWrite(RED_LED, LOW);
    digitalWrite(GRN_LED, HIGH);
  } else {
    digitalWrite(RED_LED, HIGH);
    digitalWrite(GRN_LED, LOW);
  }

}

