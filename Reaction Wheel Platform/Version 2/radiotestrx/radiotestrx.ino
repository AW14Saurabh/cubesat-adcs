

#include <Wire.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>




RF24 radio(19, 18);
const byte rxAddr[6] = "00001";



void setup(void){
  //Set up pins


  //Initialise Radio
  radio.begin();
  radio.openReadingPipe(0, rxAddr);  
  radio.startListening();

  Serial.begin(9600);
}




void loop(void){

  //Check for any radio messages
  if (radio.available()){
    float rx;
    radio.read(&rx, sizeof(rx));
    Serial.println(rx);
  }

  delay(40);
}



