//Author: Mark Yeo
//Last Modified: 2018-05-18
//Program to receive messages from Ctrl Box v1
// - prints out messages rx'd from Ctrl Box v1
// - turns on/off LEDs based on rx'd messages


// Note: blue eBay uSD card modules don't implement CS properly for MISO (MISO becomes high impedance when CS is high)
//    To fix - the SN74LVC's pin 13 (MISO enable') needs to be lifted off the PCB (PCB grounds it) and connected to pin 8 (3v3-levelled CS)
//    Pics:   https://forum.arduino.cc/index.php?PHPSESSID=ad30vsgq95h1dmu6gnp0m0udi2&action=dlattach;topic=360718.0;attach=182222
//            https://forum.arduino.cc/index.php?PHPSESSID=ad30vsgq95h1dmu6gnp0m0udi2&action=dlattach;topic=360718.0;attach=182224



#include <Wire.h>
#include <SPI.h>
#include <RF24.h>
#include <SD.h>

#define ARDUINO_COMMS  9
#define LED_G 6
#define LED_B 9
#define SD_CS 16
#define NRF_CE 15
#define NRF_CS 14
#define DISABLED 0
#define ENABLED 1
#define DETUMBLE 0
#define POINT 1
#define FILE_NAME "OBCdata.txt"


RF24 radio(NRF_CE, NRF_CS);
const byte rxAddr[6] = "00002";
File dataFile;



void setup(void){
  //=================================
  // INITIALISATION
  //=================================
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  pinMode(NRF_CE, OUTPUT);
  pinMode(NRF_CS, OUTPUT);
  pinMode(SD_CS, OUTPUT);
  digitalWrite(LED_G, LOW);
  digitalWrite(LED_B, LOW);
  digitalWrite(NRF_CS, LOW);
  digitalWrite(SD_CS, HIGH);
  
  Serial.begin(9600);
  Wire.begin();
  radio.begin();
  radio.openReadingPipe(0, rxAddr);  
  radio.startListening();

  
  SD.begin(SD_CS);

  
  //=================================
  // STARTUP ROUTINE
  //=================================
  dataFile = SD.open(FILE_NAME, FILE_WRITE);
  if(dataFile){
    dataFile.println();
    dataFile.println("Comms Startup ");
    dataFile.println(millis());
    dataFile.close();
  } else {
    return;
  }
  
  // Ready!
  digitalWrite(LED_G, HIGH);
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
  unsigned long currentMillis = millis();
  Serial.print(currentMillis);
  Serial.print(" ");

  //Check for any radio messages
  if (radio.available()){
    struct packet rx;
    radio.read(&rx, sizeof(rx));

    if (rx.h == 'a'){
      angleTarg = rx.d * (2*PI) / 360.0; //updated to reflect ctrl box sending angle of pot measured around z+ axis (CCW)
                      //^ '//' ONLY FOR DEBUGGING

    } else if (rx.h == 'm'){
      mode = rx.d;
    } else if (rx.h == 'e'){
      motorState = rx.d;
    }
    i = 0; //radio 'watchdog' reset - see below

    if (mode == DETUMBLE){
      digitalWrite(LED_B, LOW);
    } else {
      digitalWrite(LED_B, HIGH);
    }
    if (motorState == DISABLED){
      digitalWrite(LED_G, LOW);
    } else {
      digitalWrite(LED_G, HIGH);
    }
    //analogWrite(LED_B, angleTarg*255.0/(2*PI));

    Serial.print(angleTarg);
    Serial.print(" ");
    Serial.print(mode);
    Serial.print(" ");
    Serial.print(motorState);
  } else {
    Serial.print("No messages RX'd ");
  }


  dataFile = SD.open(FILE_NAME, FILE_WRITE);
  if(dataFile){
    dataFile.print(currentMillis);
    dataFile.print(" ");
    dataFile.print(angleTarg);
    dataFile.print(" ");
    dataFile.print(mode);
    dataFile.print(" ");
    dataFile.println(motorState);
    dataFile.close();
    Serial.print("data logged ");
  } else {
    Serial.print("SD error ");
  }

  Serial.println();

  delay(updateTime);
  
}



