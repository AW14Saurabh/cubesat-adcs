#include <Wire.h>
#include <FaBo9Axis_MPU9250.h>
#include "I2Cdev.h"
#include "RTClib.h"
#include "MPU6050.h"


#define address 0x1E //0011110b, I2C 7bit address of HMC5883
#define MPU6_CS 8
#define MPU9_CS 9

DS1307 rtc;
FaBo9Axis fabo_9axis;
MPU6050 accelgyro(0x69);


void setup() {

  
  pinMode(MPU6_CS, OUTPUT);
  pinMode(MPU9_CS, OUTPUT);
  
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif
  
  Serial.begin(9600);
  //Wire.begin();

  Serial.println("Begin Sensor Setup");

  digitalWrite(MPU6_CS, LOW);
  digitalWrite(MPU9_CS,HIGH);
  if (fabo_9axis.begin()) {
    Serial.println("MPU9250 online");
    fabo_9axis.setMPU(0x69); //set address of MPU9
  } else {
    Serial.println("MPU9250 device error");
    while(1);
  }
  digitalWrite(MPU9_CS, LOW);
  digitalWrite(MPU6_CS, HIGH);
  accelgyro.initialize();
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");


  //Initialise HMC5883L
  //Put the HMC5883 IC into the correct operating mode
  Wire.beginTransmission(address); //open communication with HMC5883
  Wire.write(0x02); //select mode register
  Wire.write(0x00); //continuous measurement mode
  Wire.endTransmission();

  //Start the timer
  rtc.begin();
  if (rtc.isrunning()) {
    Serial.println("RTC online");
    rtc.adjust(DateTime(__DATE__, __TIME__));
  } else {
    Serial.println("RTC is NOT running!");
  }
}


void loop() {
  //=============
  // MPU9250 CODE
  //=============
  Serial.println("---MPU9 data---");
  float ax,ay,az;
  float gx,gy,gz;
  float mx,my,mz;
  float temp;
  Serial.println(fabo_9axis.getMPU()); //prints address of MPU

  //Poll sensor
  digitalWrite(MPU6_CS, LOW); //change from 6 to 9
  digitalWrite(MPU9_CS, HIGH);   // turn the LED on (HIGH is the voltage level)
  fabo_9axis.readAccelXYZ(&ax,&ay,&az);
  fabo_9axis.readGyroXYZ(&gx,&gy,&gz);
  fabo_9axis.readMagnetXYZ(&mx,&my,&mz);
  fabo_9axis.readTemperature(&temp);
  digitalWrite(MPU9_CS,LOW);
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

  //=============
  // MPU6050 CODE
  //=============
  int16_t ax1,ay1,az1;
  int16_t gx1,gy1,gz1;
  Serial.println("---MPU6 data---");
  accelgyro.getMotion6(&ax1, &ay1, &az1, &gx1, &gy1, &gz1);


  //Print sensor data
  String mpu6accel = "ax: " + String(ax1,DEC) + " ay: " + String(ay1,DEC) + " az: " + String(az1,DEC);
  String mpu6gyro = "gx: " + String(gx1,DEC) + " gy: " + String(gy1,DEC) + " gz: " + String(gz1,DEC);
  Serial.println(mpu6accel);
  Serial.println(mpu6gyro);

  digitalWrite(MPU9_CS, HIGH);   // turn the LED on (HIGH is the voltage level)


  //=============
  // HMC5883L CODE
  //=============
  int x,y,z; //triple axis data
  Serial.println("---HMC5883L data---");
  
  //Tell the HMC5883L where to begin reading data
  Wire.beginTransmission(address);
  Wire.write(0x03); //select register 3, X MSB register
  Wire.endTransmission();
 
 //Read data from each axis, 2 registers per axis
  Wire.requestFrom(address, 6);
  if (6 <=Wire.available()){
     x = Wire.read()<<8; //X msb
     x |= Wire.read(); //X lsb
     z = Wire.read()<<8; //Z msb
     z |= Wire.read(); //Z lsb
     y = Wire.read()<<8; //Y msb
     y |= Wire.read(); //Y lsb
  }
  //Print out values of each axis
  String HMC5accel= "ax: " + String(x) + " ay: " + String(y) + " az: " + String(z);
  Serial.println(HMC5accel);

  //=========
  // RTC CODE
  //========= 
  Serial.println("---Timer--");
  DateTime now = rtc.now();
  char buf[100];
  strncpy(buf,"DD.MM.YYYY  hh:mm:ss\0",100);
  Serial.println(now.format(buf));

  delay(100);
}
