//Mark Yeo; mark.yeo@student.unsw.edu.au
//Last Modified 2018-04-26
//Program to test ctrl box v2 radio (RX)
// - prints out messages rx'd from ctrl box v1 (w/ structs)
// - turns on/off LEDs based on rx'd messages
// (note that ctrl-box RGB LED has a common +ve lead
//   so LEDs turn on when relevant pin is pulled LOW)

#include <Wire.h>
#include <SPI.h>
#include <RF24.h>

#define LED_R 5
#define LED_G 6
#define LED_B 9
#define SW_L  8
#define SW_R  4
#define PB    7
#define NRF_CE 14
#define NRF_CS 15
#define DETUMBLE 0
#define POINT 1
#define DISABLED 0
#define ENABLED 1


RF24 radio(NRF_CE, NRF_CS);
const byte rxAddr[6] = "00001";



void setup(void){
  //Set up pins
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  pinMode(SW_L, INPUT);
  pinMode(SW_R, INPUT);
  pinMode(PB, INPUT);
  
  //Initialise Radio
  radio.begin();
  radio.openReadingPipe(0, rxAddr);  
  radio.startListening();

  Serial.begin(9600);

  // Ready!
  //digitalWrite(LED_R, HIGH);
  //digitalWrite(LED_G, HIGH);
  //digitalWrite(LED_B, HIGH);
}



struct packet{
  char h;
  int d;
};

int updateTime = 50; //ms
float angleTarg = 0;
int mode = DETUMBLE;
int motorState = DISABLED;
int i = 0;

void loop(void){

  //Check for any radio messages
  if (radio.available()){
    struct packet rx;
    radio.read(&rx, sizeof(rx));

    if (rx.h == 'a'){
      angleTarg = rx.d * 2*PI / 360.0; //updated to reflect ctrl box sending angle of pot measured around z+ axis (CCW)

    } else if (rx.h == 'm'){
      mode = rx.d;
    } else if (rx.h == 'e'){
      motorState = rx.d;
    }
    i = 0; //radio 'watchdog' reset - see below

    if (mode == DETUMBLE){
      digitalWrite(LED_R, LOW);
    } else {
      digitalWrite(LED_R, HIGH);
    }
    if (motorState == DISABLED){
      digitalWrite(LED_G, LOW);
    } else {
      digitalWrite(LED_G, HIGH);
    }
    if (angleTarg > PI){
      digitalWrite(LED_B, LOW);
    } else {
      digitalWrite(LED_B, HIGH);
    }
    Serial.print(angleTarg);
    Serial.print(" ");
    Serial.print(mode);
    Serial.print(" ");
    Serial.println(motorState);
  }
  delay(updateTime);
}



