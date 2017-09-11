//Receives radio messages from Ctrl Box v1

#include <Wire.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define VR 5
#define ZF 7

#define CCW 0
#define CW 1

#define RED_LED 8
#define GREEN_LED 9


//RF24 radio(7,8); //Ctrl Box v1
//RF24 radio(18,19); //RWSv1
RF24 radio(19,18); //RWSv2
const byte rxAddr[6] = "00001";


void setup(void){
  //Set up pins
  pinMode(9, OUTPUT);


  //Initialise Radio
  radio.begin();
  radio.openReadingPipe(0, rxAddr);  
  radio.startListening();

  Serial.begin(9600);
  Serial.println("starting radio");
}


struct packet{
  char h;
  int d;
};
int angle = 0;
int mode = 0;
int enable = 0;

void loop(void){

  //Check for any radio messages
  if (radio.available()){
    struct packet rx;
    radio.read(&rx, sizeof(rx));

    if (rx.h == 'a'){
      angle = rx.d;
    } else if (rx.h == 'm'){
      mode = rx.d;
    } else if (rx.h == 'e'){
      enable = rx.d;
    }
    Serial.print("Angle: ");
    Serial.print(angle);
    Serial.print(" Mode: ");
    Serial.print(mode);
    Serial.print(" Enable: ");
    Serial.println(enable);

  }
  


  if (angle > 255){
    angle = 255;
  } else if (angle < 0){
    angle = 0;
  }

  if (enable){
    analogWrite(VR, angle);
  }
  if (mode == CW){
    digitalWrite(ZF, LOW);
    Serial.print("ZF LOW ");
    Serial.println(mode);
  } else {
    digitalWrite(ZF, HIGH);
    Serial.print("ZF HI ");
    Serial.println(mode);
  }

  
  delay(200);
}




