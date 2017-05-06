//Mark Yeo; mark.yeo@student.unsw.edu.au
//Last Modified 2017-02-14
//Reaction Wheel System v1 - RF Control Box PID Tuner
//This program is used to tune the v1 PID controller


#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#define POT_PIN A0
#define DET_PT_TOGGLE A1
#define ACTIVATE_MOTOR A2

RF24 radio(9, 10);
const byte rxAddr[6] = "00001";

void setup()
{
  Serial.begin(115200);
  radio.begin();
  radio.setRetries(15, 15);
  radio.openWritingPipe(rxAddr);
  radio.stopListening();
}

void loop()
{
  int swi0 = digitalRead(A0);
  int pot1 = analogRead(A1);
  int pot2 = analogRead(A2);
  int pot3 = analogRead(A3);
  int swi4 = digitalRead(A4);

  float tx0 = (float)swi0;  //motor enable
  float tx1 = 20.0 + (float)pot1 /1024.0*5.0; //kP
  float tx2 = 40.0 + (float)pot2 /1024.0*5.0; //kI
  float tx3 = 60.0 + (float)pot3 /1024.0*5.0; //kD
  float tx4 = 80.0 + (float)swi4 * PI; //0 or 180deg target

  
  /*
  Serial.println(swi0);
  Serial.println(pot1);
  Serial.println(pot2);
  Serial.println(pot3);
  */

  
  radio.write(&tx0, sizeof(tx0));
  radio.write(&tx1, sizeof(tx1));
  radio.write(&tx2, sizeof(tx2));
  radio.write(&tx3, sizeof(tx3));
  radio.write(&tx4, sizeof(tx4));

  
  Serial.println(tx0);
  Serial.println(tx1-20.0);
  Serial.println(tx2-40.0);
  Serial.println(tx3-60.0);
  Serial.println(tx4-80.0);
  Serial.println();

  delay(10);
}
