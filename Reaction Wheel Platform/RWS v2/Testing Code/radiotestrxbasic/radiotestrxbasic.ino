//Receives radio messages from Ctrl Box v1

#include <Wire.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>




//RF24 radio(7,8); //ctrl box v1
//RF24 radio(19,18); //RWSv2
RF24 radio(18,19); //RWSv1
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
  digitalWrite(9, LOW);

}


int i = 0;


void loop(void){

  //Check for any radio messages
  if (radio.available()){
    float rx;
    radio.read(&rx, sizeof(rx));
    Serial.print("data in:");
    Serial.print(rx);
    Serial.print(" iteration: ");
    Serial.println(i);
    i++;
    if (rx > 10.0){
        digitalWrite(9, HIGH);
    } else{
        digitalWrite(9, LOW);
    }
  }

  delay(40);
}



