#include <Wire.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "MPU9250.h"

MPU9250 myIMU;

float biasZ = 0;
const byte rxAddr[6] = "00001";

void setup(){
  Wire.begin();
  Serial.begin(9600);
  delay(3000);
  Serial.println("startprog");
  // Calibrate gyro and accelerometers, load biases in bias registers
  myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);
  myIMU.initMPU9250();
  
  // Calibrate Gyro Manually
  int calibIters = 100;
  int calibTime = 10;
  int i = 0;
  for (i = 0; i < calibIters; i++){
    myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values
    myIMU.getGres();
    biasZ += (float)myIMU.gyroCount[2]*myIMU.gRes;
    delay(calibTime);
  }
  biasZ /= calibIters;
  Serial.println("startradio()");

  RF24 radio(18, 19); //swap maybe?

  radio.setRetries(15, 15);
//  radio.openWritingPipe(rxAddr);
//  radio.stopListening();
  Serial.println("endsetup");

}


int updTime = 100;
float angle = 0;

void loop(){
  myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values
  myIMU.getGres();
  float wZ = (float)myIMU.gyroCount[2]*myIMU.gRes - biasZ;
  Serial.print("wZ = "); Serial.println(wZ, 2);

  angle += wZ*(updTime/1000.0)*8.0;  //when arduino pro nano starts up without USB, it runs ~8x slower (it's a firmware bug)  
  Serial.print("angle = "); Serial.println(angle, 2);

  float tx = angle;
//  radio.write(&tx, sizeof(tx));
  delay(updTime);
}
