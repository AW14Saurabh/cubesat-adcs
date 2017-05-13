#include <Wire.h>
#include <FaBo9Axis_MPU9250.h>
#include "RTClib.h"

#define MPU6_CS 8
#define MPU9_CS 9


DS1307 rtc;


FaBo9Axis fabo_9axis;
void setup() {
  pinMode(MPU6_CS, OUTPUT);
  pinMode(MPU9_CS, OUTPUT);
  digitalWrite(MPU9_CS, HIGH);
  digitalWrite(MPU6_CS, HIGH);

  Serial.begin(9600);
  Wire.begin();

  Serial.println("Begin Sensor Setup");

  digitalWrite(MPU6_CS, LOW);
  if (fabo_9axis.begin()) {
    Serial.println("MPU9250 online");
    fabo_9axis.setMPU(0x69); //set address of MPU9
  } else {
    Serial.println("MPU9250 device error");
    while(1);
  }
  digitalWrite(MPU6_CS, HIGH);

  rtc.begin();
  if (rtc.isrunning()) {
    Serial.println("RTC online");
    rtc.adjust(DateTime(__DATE__, __TIME__));
  } else {
    Serial.println("RTC is NOT running!");
  }
}

int flag = 1;


void loop() {

  //=============
  // MPU9250 CODE
  //=============
  
  float ax,ay,az;
  float gx,gy,gz;
  float mx,my,mz;
  float temp;
  Serial.println(fabo_9axis.getMPU()); //printsaddress of MPU

  //Poll sensor
  digitalWrite(MPU6_CS, LOW);
  fabo_9axis.readAccelXYZ(&ax,&ay,&az);
  fabo_9axis.readGyroXYZ(&gx,&gy,&gz);
  fabo_9axis.readMagnetXYZ(&mx,&my,&mz);
  fabo_9axis.readTemperature(&temp);
  digitalWrite(MPU6_CS, HIGH);

  //Print sensor data
  String mpu9accel = "ax: " + String(ax,3) + " ay: " + String(ay,3) + " az: " + String(az,3);
  String mpu9gyro = "gx: " + String(gx,3) + " gy: " + String(gy,3) + " gz: " + String(gz,3);
  String mpu9mag = "mx: " + String(mx,3) + " my: " + String(my,3) + " mz: " + String(mz,3);
  String mpu9temp = "temp: " + String(temp,3);
  Serial.println(mpu9accel);
  Serial.println(mpu9gyro);
  Serial.println(mpu9mag);
  Serial.println(mpu9temp);




  //=========
  // RTC CODE
  //=========

  DateTime now = rtc.now();
  char buf[100];
  strncpy(buf,"DD.MM.YYYY  hh:mm:ss\0",100);
  Serial.println(now.format(buf));


  delay(100);
}
