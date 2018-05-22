// Author: Mark Yeo
// Last Modified 2018-05-21
// Program for PWR Board Arduino. Does the following:
// - requests data from comms arduino
// - requests data from gyro arduino
// - requests data from rws arduino
// - sends gyro + comms data onto rws arduino
// - sends gyro + rws data to comms arduino

#include <Wire.h>

#define RWS_I  A3
#define RWS_V  A2

#define ARDUINO_GYRO  1
#define ARDUINO_COMMS  2
#define ARDUINO_RWS   3

#define NUM_GYRO_RX_FLOATS 7
#define NUM_RWS_TX_FLOATS 7
#define NUM_COMMS_RX_UINTS 1
#define NUM_RWS_TX_UINTS 1
#define NUM_RWS_RX_INTS 4
#define NUM_PWR_TX_INTS 2
#define SIZE_FLOAT 4
#define SIZE_UINT 2
#define SIZE_INT 2
#define TRANSMISSION_0 0
#define TRANSMISSION_1 1



union uByteSevenFloat {
 byte b[NUM_GYRO_RX_FLOATS*SIZE_FLOAT];
 float val[NUM_GYRO_RX_FLOATS];
} uGyroRx;

union uByteOneUInt {
 byte b[NUM_COMMS_RX_UINTS*SIZE_UINT];
 unsigned int val;
} uCommsRx;


union uByteFourInt {
 byte b[NUM_RWS_RX_INTS*SIZE_INT];
 int val[NUM_RWS_RX_INTS];
} uRwsRx;


union uByteTwoInt {
  byte b[NUM_PWR_TX_INTS*SIZE_INT];
  int val[NUM_PWR_TX_INTS];
} uPwrTx;






void setup() {
  Wire.begin();        // join i2c bus (address optional for master)
  Serial.begin(9600);  // start serial for output
}



void loop() {
  int i;
  byte data[100]; //big array to store input bytes

  // Request Data from Comms Arduino
  Wire.requestFrom(ARDUINO_COMMS, NUM_COMMS_RX_UINTS*SIZE_UINT);    // request 2 bytes from Comms Arduino
  i = 0;
  while (Wire.available()) {
    data[i] = Wire.read(); // receive a byte as character
    i = i + 1;
  }
  for (i=0; i<NUM_COMMS_RX_UINTS*SIZE_UINT; i++){
    uCommsRx.b[i] = data[i];
  }

  // Request Data from Gyro Arduino
  Wire.requestFrom(ARDUINO_GYRO, NUM_GYRO_RX_FLOATS*SIZE_FLOAT);    // request 7 floats from Attitude Arduino
  i = 0;
  while (Wire.available()) {
    data[i] = Wire.read(); // receive a byte as character
    i = i + 1;
  }
  for (i=0; i<NUM_GYRO_RX_FLOATS*SIZE_FLOAT; i++){
    uGyroRx.b[i] = data[i];
  }


  // Request Data from RWS Arduino
  Wire.requestFrom(ARDUINO_RWS, NUM_RWS_RX_INTS*SIZE_INT);    // request 4 ints from RWS Arduino (4x analogIns measuring wheel freq)
  i = 0;
  while (Wire.available()) {
    data[i] = Wire.read();
    i = i + 1;
  }
  for (i=0; i<NUM_RWS_RX_INTS*SIZE_INT; i++){
    uRwsRx.b[i] = data[i];
  }

  


  // Send Data to RWS Arduino
  Wire.beginTransmission(ARDUINO_RWS);
  for (i=0; i<NUM_GYRO_RX_FLOATS*SIZE_FLOAT; i++){  //forward data from gyro arduino to rws arduino
    Wire.write(uGyroRx.b[i]);
  }
  for (i=0; i<NUM_COMMS_RX_UINTS*SIZE_UINT; i++){  //forward data from comms arduino to rws arduino
    Wire.write(uCommsRx.b[i]);
  }
  Wire.endTransmission();







  int currentCumu = 0;
  int voltageCumu = 0;
  for (i=0; i<10; i++){
    currentCumu += analogRead(RWS_I);
    voltageCumu += analogRead(RWS_V);
    delay(5);
  }
  int current = currentCumu/10;
  int voltage = voltageCumu/10;

  uPwrTx.val[0] = current;
  uPwrTx.val[1] = voltage;



  // Send Data to Comms Arduino
  Wire.beginTransmission(ARDUINO_COMMS); // transmit to Comms Arduino
  Wire.write(TRANSMISSION_0);
  for (i=0; i<NUM_GYRO_RX_FLOATS*SIZE_FLOAT; i++){
    Wire.write(uGyroRx.b[i]);
  }
  Wire.endTransmission();
  
  Wire.beginTransmission(ARDUINO_COMMS); // transmit to Comms Arduino
  Wire.write(TRANSMISSION_1);
  for (i=0; i<NUM_RWS_RX_INTS*SIZE_INT; i++){
    Wire.write(uRwsRx.b[i]);
  }
  for (i=0; i<NUM_PWR_TX_INTS*SIZE_INT; i++){
    Wire.write(uPwrTx.b[i]);
  }
  Wire.endTransmission();

//if i2c values are all messed up, check you have the right address + you're sending/rx'ing 32 bytes or less








  //Debug messages
  for (i=0; i<NUM_GYRO_RX_FLOATS; i++){
    Serial.print(uGyroRx.val[i]);
    Serial.print(" ");
  }
  Serial.print("     ");
  Serial.print(uCommsRx.val);
  Serial.print("     ");
  for (i=0; i<NUM_RWS_RX_INTS; i++){
    Serial.print(uRwsRx.val[i]);
    Serial.print(" ");
  }
  Serial.println();

  delay(100);
}
