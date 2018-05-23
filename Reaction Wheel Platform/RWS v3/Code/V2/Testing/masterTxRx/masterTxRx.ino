
#include <Wire.h>

void setup() {
  Wire.begin(); // join i2c bus (address optional for master)
  Serial.begin(9600);  // start serial for output
}

char x = 'a';

void loop() {
  Wire.beginTransmission(12); // transmit to device #8
  Wire.write(x);
  Wire.endTransmission();    // stop transmitting
  Wire.requestFrom(12, 1);    // request 6 bytes from slave device #8
  Serial.print("sending: ");
  Serial.println(x);
  while (Wire.available()) { // slave may send less than requested
    char c = Wire.read(); // receive a byte as character
    Serial.println(c);         // print the character
  }
  x++;
  delay(500);
}
