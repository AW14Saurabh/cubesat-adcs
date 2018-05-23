
#include <Wire.h>

void setup() {
  Wire.begin(12);                // join i2c bus with address #8
  Wire.onReceive(receiveEvent); // register event
  Wire.onRequest(requestEvent); // register event
  Serial.begin(9600);           // start serial for output
}

char rx;

void loop() {
  delay(100);
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany) {
  while (Wire.available()) { // loop through all but the last
    rx = Wire.read(); // receive byte as a character
    Serial.print(rx);         // print the character
  }
}

void requestEvent() {
  Wire.write(rx+1); // respond with message of 6 bytes
  // as expected by master
}
