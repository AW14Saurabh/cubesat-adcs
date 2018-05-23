//Mark Yeo; mark.yeo@student.unsw.edu.au
//Last Modified 2018-04-26
//Program to test OBC+Comms components
// - prints out RTC time
// - if received, prints out messages rx'd from ctrl box v1 (w/ structs)
// - turns on/off LEDs based on rx'd messages

// Note: the SPI bus can't handle more than 1 slave device (probably needs tri-state buffers)
//  so the uSD card module must not be connected (physically) for the radio to work


#include <Wire.h>
#include <SPI.h>
#include <RF24.h>

#define NRF_CE 15
#define NRF_CS 14

RF24 radio(NRF_CE, NRF_CS); //#define these
const byte rxAddr[6] = "00020";

void setup(void){
  pinMode(NRF_CE, OUTPUT);
  pinMode(NRF_CS, OUTPUT);
  Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(0, rxAddr);  
  radio.startListening();
}



int updateTime = 50; //ms
void loop(void){
  if (radio.available()){
    unsigned int messageIn;
    radio.read(&messageIn, sizeof(messageIn));
    Serial.print(messageIn);
    Serial.println();
  }
  delay(updateTime);
}



