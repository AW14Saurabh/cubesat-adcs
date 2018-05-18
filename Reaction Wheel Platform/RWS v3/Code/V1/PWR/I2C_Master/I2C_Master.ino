
#define ARDUINO_GYRO  8

#include <Wire.h>

void setup() {
  Wire.begin();        // join i2c bus (address optional for master)
  Serial.begin(9600);  // start serial for output
}

union u_tag {
 byte b[4];
 float fval;
} u;

void loop() {
  Wire.requestFrom(ARDUINO_GYRO, 60);    // request 6 bytes from slave device #8
  int i = 0;
  byte data[100];
  while (Wire.available()) { // slave may send less than requested
    data[i] = Wire.read(); // receive a byte as character
    i = i + 1;
  }
  for (i=0; i<4; i++){
    u.b[i] = data[i];
  }
  Serial.println(u.fval);         // print the character

  delay(500);
}
