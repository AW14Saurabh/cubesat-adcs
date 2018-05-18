//Mark Yeo; mark.yeo@student.unsw.edu.au
//Last Modified 2018-04-26
//Program to test OBC+Comms components
// - prints out RTC time
// - if received, prints out messages rx'd from ctrl box v1 (w/ structs)
// - turns on/off LEDs based on rx'd messages

// Note: the uSD card module doesn't implement CS properly for MISO (MISO becomes high impedance when CS is high)
//    To fix - the SN74LVC's pin 13 (MISO enable') needs to be lifted off the PCB (PCB grounds it) and connected to pin 8 (3v3-levelled CS)
//    Pics:   https://forum.arduino.cc/index.php?PHPSESSID=ad30vsgq95h1dmu6gnp0m0udi2&action=dlattach;topic=360718.0;attach=182222
//            https://forum.arduino.cc/index.php?PHPSESSID=ad30vsgq95h1dmu6gnp0m0udi2&action=dlattach;topic=360718.0;attach=182224

#include <Wire.h>
#include <SPI.h>
#include <RF24.h>
#include <RTClib.h>
#include <SD.h>

#define LED_Y 5
#define LED_G 6
#define LED_B 9
#define PB    7
#define SD_CS 2
#define NRF_CE 15
#define NRF_CS 14
#define RTC_CLK 17
#define DETUMBLE 0
#define POINT 1
#define DISABLED 0
#define ENABLED 1
#define FILE_NAME "OBCdata.txt"


DS3231 rtc; //if garbage values (e.g. "20@5-25-@5  @5:@5:85"), make sure Wire.begin(); is run + no other devices at 0x68
RF24 radio(NRF_CE, NRF_CS); //#define these
const byte rxAddr[6] = "00001";
File dataFile;



void setup(void){
  //=================================
  // INITIALISATION
  //=================================
  pinMode(LED_Y, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  pinMode(NRF_CE, OUTPUT);
  pinMode(NRF_CS, OUTPUT);
  pinMode(SD_CS, OUTPUT);
  pinMode(PB, INPUT);
  digitalWrite(LED_Y, HIGH);
  digitalWrite(LED_G, LOW);
  digitalWrite(LED_B, LOW);
  digitalWrite(NRF_CS, HIGH);
  digitalWrite(SD_CS, HIGH);
  
  Serial.begin(9600);
  Wire.begin();
  rtc.begin();  // Initialise DS3231  (Real Time Clock)
  //rtc.adjust(DateTime(__DATE__, __TIME__)); //use this to set RTC time //as of 2018-04-22 RTC clock is 4s slow
  if (!rtc.isrunning()) { //if the coin battery has run out, then reset the clock to midnight 1st Jan 2000
    rtc.adjust(DateTime(0, 1, 1, 0, 0, 0));
  }
  radio.begin();
  radio.openReadingPipe(0, rxAddr);  
  radio.startListening();
  SD.begin(SD_CS);

  
  //=================================
  // STARTUP ROUTINE
  //=================================
  DateTime now = rtc.now();
  char buf[25];
  strncpy(buf,"YYYY-MM-DD hh:mm:ss\0",25);
  dataFile = SD.open(FILE_NAME, FILE_WRITE);
  if(dataFile){
    dataFile.println();
    dataFile.println("OBC Startup ");
    dataFile.print(now.format(buf));
    dataFile.print(" ");
    dataFile.println(millis());
    dataFile.close();
  } else {
    return;
  }
  
  // Ready!
  //digitalWrite(LED_Y, LOW);
  //digitalWrite(LED_G, HIGH);
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
  DateTime now = rtc.now();
  char buf[25];
  strncpy(buf,"YYYY-MM-DD hh:mm:ss \0",25);
  Serial.print(now.format(buf));

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
      digitalWrite(LED_Y, LOW);
    } else {
      digitalWrite(LED_Y, HIGH);
    }
    if (motorState == DISABLED){
      digitalWrite(LED_G, LOW);
    } else {
      digitalWrite(LED_G, HIGH);
    }
    analogWrite(LED_B, angleTarg*255.0/(2*PI));
    /*
    if (angleTarg > PI){
      digitalWrite(LED_B, LOW);
    } else {
      digitalWrite(LED_B, HIGH);
    }*/
    Serial.print(angleTarg);
    Serial.print(" ");
    Serial.print(mode);
    Serial.print(" ");
    Serial.print(motorState);
  }
  Serial.println();


  dataFile = SD.open(FILE_NAME, FILE_WRITE);
  if(dataFile){
    dataFile.print(now.format(buf));
    dataFile.print(" ");
    dataFile.print(millis());
    dataFile.print(" ");
    dataFile.print(angleTarg);
    dataFile.print(" ");
    dataFile.print(mode);
    dataFile.print(" ");
    dataFile.println(motorState);
    dataFile.close();
    Serial.println("data logged");
  } else {
    Serial.println("error accessing file");
  }

  
  delay(updateTime);
  
}



