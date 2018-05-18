
#include <Wire.h>

#define ARDUINO_GYRO  8
#define ARDUINO_COMMS  10


void setup() {
  Wire.begin();        // join i2c bus (address optional for master)
  Serial.begin(9600);  // start serial for output
}

union u_byte_float {
 byte b[28];
 float val[7];
} uAtt;

int uAttNumFloats = 7;

union u_byte_uint {
 byte b[2];
 unsigned int val;
} uComms;


void loop() {
  int i;
  byte data[100]; //big array to store input bytes


  Wire.requestFrom(ARDUINO_GYRO, uAttNumFloats*4);    // request 40 bytes from Attitude Arduino
  i = 0;
  while (Wire.available()) { // slave may send less than requested
    data[i] = Wire.read(); // receive a byte as character
    i = i + 1;
  }
  for (i=0; i<uAttNumFloats*4; i++){
    uAtt.b[i] = data[i];
  }

  Wire.requestFrom(ARDUINO_COMMS, 2);    // request 2 bytes from Comms Arduino
  //char c = Wire.read(); // receive a byte as character
  //char d = Wire.read(); // receive a byte as character
  i = 0;
  while (Wire.available()) { // slave may send less than requested
    data[i] = Wire.read(); // receive a byte as character
    i = i + 1;
  }
  for (i=0; i<2; i++){
    uComms.b[i] = data[i];
  }
  Wire.beginTransmission(ARDUINO_COMMS); // transmit to Comms Arduino
  for (i=0; i<uAttNumFloats*4; i++){
    Wire.write(uAtt.b[i]);
  }
  Wire.endTransmission();    // stop transmitting


  for (i=0; i<uAttNumFloats; i++){
    Serial.print(uAtt.val[i]);
    Serial.print(" ");
  }
  Serial.print(uComms.val);
  Serial.println();

  delay(100);
}
