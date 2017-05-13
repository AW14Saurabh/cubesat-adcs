//Mark Yeo; mark.yeo@student.unsw.edu.au
//Last Modified 2017-02-14
//Reaction Wheel System v1 - RF Control Box Code
//This program reads in the angle of a potentiometer and the state of an activation switch,
// then sends the data to the reaction wheel via an NRF24L01 chip


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
  Serial.begin(9600);
  radio.begin();
  radio.setRetries(15, 15);
  radio.openWritingPipe(rxAddr);
  radio.stopListening();
}

void loop()
{
  int mode = digitalRead(DET_PT_TOGGLE);
  int motorActivate = digitalRead(ACTIVATE_MOTOR);
  float potPcnt = analogRead(POT_PIN)/1024.0;         //Pot percentage turned
  float potAngle = potPcnt * (2*PI) * (300.0/360.0);  //Angle pot turned (radians) - calibrated to a pot that only turns 300deg
  float tx = motorActivate*2*(2*PI) + mode*(2*PI) + potAngle; //We want to send all of our data in one number, so we add 2Pi to the angle if the switch is turned on
                                                              //E.g. if pot is at 10deg and switchA is off, tx is 10deg. if switch A is on, tx is 370deg
  Serial.println(tx);
  radio.write(&tx, sizeof(tx));
  delay(100);
}
