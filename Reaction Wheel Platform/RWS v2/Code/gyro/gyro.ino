//Program to detumble/point Reaction Wheel System v2 using Control Box
//Mark Yeo; mark.yeo@student.unsw.edu.au
//Last Modified 2017-09-16

//Libraries
#include <Wire.h>
#include <FaBo9Axis_MPU9250.h>


//Setup + Global Variables
FaBo9Axis fabo_9axis;


void setup(){
  Serial.begin(9600);
  fabo_9axis.begin();
  fabo_9axis.configMPU9250(MPU9250_GFS_1000, MPU9250_AFS_2G); //MPU9250_GFS_250/500/1000(~3rev/s)/2000 (max deg/s)
}



void loop(){
  //=== Get data from gyro ===
  float gx,gy,gz;
  fabo_9axis.readMagnetXYZ(&gx,&gy,&gz);
  Serial.print(String(gx,6) + "," + String(gy,6) + "," + String(gz,6) + ";");
  //Serial.println();
  delay(10);
}




