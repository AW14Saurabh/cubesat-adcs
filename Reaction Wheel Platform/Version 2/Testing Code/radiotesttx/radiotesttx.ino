//Mark Yeo; mark.yeo@student.unsw.edu.au
//Last Modified 2017-02-14
//Reaction Wheel System v1 - RF Control Box Code
//This program reads in the angle of a potentiometer and the state of an activation switch,
// then sends the data to the reaction wheel via an NRF24L01 chip


#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

//RF24 radio(18, 19);
RF24 radio(9, 10);
const byte rxAddr[6] = "00001";

void setup()
{
  Serial.begin(9600);
  radio.begin();
  radio.setRetries(15, 15);
  radio.openWritingPipe(rxAddr);
  radio.stopListening();
}

struct packet{
  char h;
  int d;
};

struct packet tx;

void loop()
{
  tx.h = 'a';
  tx.d = 1;
  while (1){
    radio.write(&tx, sizeof(tx));
    tx.h++;
    tx.d++;
    
    delay(100);
  }
}
