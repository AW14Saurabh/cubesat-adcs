//Mark Yeo; mark.yeo@student.unsw.edu.au
//Last Modified 2017-02-14
//Reaction Wheel System v1 - RF Control Box Code
//This program reads in the angle of a potentiometer and the state of an activation switch,
// then sends the data to the reaction wheel via an NRF24L01 chip


#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#define ANGLE A0
#define MODE A1
#define MOTOR_ENA A2
#define DETUMBLE 0
#define POINT 1
#define DISABLED 0
#define ENABLED 1


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
  float angle = analogRead(ANGLE)/1024.0 * 300.0;  //Angle of potentiometer (degrees) - calibrated to a pot that only turns 300deg
  angle = 300.0 - angle; //angle measured around z+ axis (CCW)
  int mode = DETUMBLE;
  if (digitalRead(MODE) == HIGH){
    mode = POINT;
  }
  int enable = DISABLED;
  if (digitalRead(MOTOR_ENA) == HIGH){
    enable = ENABLED;
  }


  tx.h = 'e';
  tx.d = enable;
  radio.write(&tx, sizeof(tx));

  if (enable == 1){
    tx.h = 'a';
    tx.d = angle;
    radio.write(&tx, sizeof(tx));
    delay(10);
  
    tx.h = 'm';
    tx.d = mode;
    radio.write(&tx, sizeof(tx));
    delay(10);
  }

  
  delay(30);
}
