//Author: Mark Yeo
//Last Modified: 2018-05-18
//Program to receive messages from Ctrl Box v2 (single int)
// - prints out messages rx'd from Ctrl Box v2
// - turns on/off LEDs based on rx'd messages
// Message structure: unsigned int ABCCCCCC CCCCCCCC where A = enableState; B = mode; C = constrain(map(yawRad, 0, PI, 0, 16383),0,16383);


// Note: blue eBay uSD card modules don't implement CS properly for MISO (MISO becomes high impedance when CS is high)
//    To fix - the SN74LVC's pin 13 (MISO enable') needs to be lifted off the PCB (PCB grounds it) and connected to pin 8 (3v3-levelled CS)
//    Pics:   https://forum.arduino.cc/index.php?PHPSESSID=ad30vsgq95h1dmu6gnp0m0udi2&action=dlattach;topic=360718.0;attach=182222
//            https://forum.arduino.cc/index.php?PHPSESSID=ad30vsgq95h1dmu6gnp0m0udi2&action=dlattach;topic=360718.0;attach=182224



#include <Wire.h>
#include <SPI.h>
#include <RF24.h>
#include <SD.h>

#define ARDUINO_COMMS  10
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
unsigned int messageIn = 0;
float wx = 0;
float wy = 0;
float wz = 0;
float q0 = 0;
float q1 = 0;
float q2 = 0;
float q3 = 0;



union u_byte_uint {
 byte b[2];
 unsigned int val;
} uComms;


union u_byte_float {
 byte b[28];
 float val[7];
} uAtt;

int uAttNumFloats = 7;

void requestEvent() {
  uComms.val = messageIn;
  int i;
  for (i=0; i<2; i++){
    Wire.write(uComms.b[i]); //2 bytes
  }
}

void receiveEvent(int bytesIn) {
  int i = 0;
  while (Wire.available()) { // slave may send less than requested
    uAtt.b[i] = Wire.read(); // receive a byte as character
    i = i + 1;
  }
  wx = uAtt.val[0];
  wy = uAtt.val[1];
  wz = uAtt.val[2];
  q0 = uAtt.val[3];
  q1 = uAtt.val[4];
  q2 = uAtt.val[5];
  q3 = uAtt.val[6];
}



void setup(void){
  //=================================
  // INITIALISATION
  //=================================
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  pinMode(NRF_CE, OUTPUT);
  pinMode(NRF_CS, OUTPUT);
  pinMode(SD_CS, OUTPUT);
  digitalWrite(LED_G, HIGH);
  digitalWrite(LED_B, HIGH);
  digitalWrite(NRF_CS, LOW);
  digitalWrite(SD_CS, HIGH);
  
  Serial.begin(9600);
  Wire.begin(ARDUINO_COMMS);                // join i2c bus with address #10
  Wire.onRequest(requestEvent); // register event
  Wire.onReceive(receiveEvent); // register event
  
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
  digitalWrite(LED_G, LOW);
  digitalWrite(LED_B, LOW);
}



int updateTime = 50; //ms
float angleTarg = 0;
int mode = DETUMBLE;
int enableState = DISABLED;
int i = 0;
bool ledBOn = false;
bool ledGOn = false;

void loop(void){
  unsigned long currentMillis = millis();

  //Check for any radio messages
  if (radio.available()){
    radio.read(&messageIn, sizeof(messageIn));


    unsigned int message = messageIn;
    enableState = message >> 15;
    message = message & ((unsigned int)B01111111*256 + B11111111);
    mode = message >> 14;
    message = message & ((unsigned int)B10111111*256 + B11111111);    
    angleTarg = constrain(message*2*PI/16383.0,0,2*PI);
    if (ledGOn){
      digitalWrite(LED_G, LOW);
      ledGOn = false;
    } else {
      digitalWrite(LED_G, HIGH);
      ledGOn = true;
    }
  }
/*
    //i = 0; //radio 'watchdog' reset - see below
    if (mode == DETUMBLE){
      digitalWrite(LED_B, LOW);
    } else {
      digitalWrite(LED_B, HIGH);
    }
    if (enableState == DISABLED){
      digitalWrite(LED_G, LOW);
    } else {
      digitalWrite(LED_G, HIGH);
    }
*/



  String strC = "c:" + String(angleTarg,4) + "," + String(mode) + "," + String(enableState) + " ";
  String strW = "w:" + String(wx,4) + "," + String(wy,4) + "," + String(wz,4) + " ";
  String strQ = "q:" + String(q0,6) + "," + String(q1,6) + "," + String(q2,6) + " " + String(q3,6) + " ";
  Serial.print(currentMillis);
  Serial.print(" ");
  Serial.print(strC);
  Serial.print(strW);
  Serial.print(strQ);
  Serial.println();


  
  dataFile = SD.open(FILE_NAME, FILE_WRITE);
  if(dataFile){
    dataFile.print(currentMillis);
    dataFile.print(" ");
    dataFile.print(strC);
    dataFile.print(strW);
    dataFile.print(strQ);
    dataFile.println();
    dataFile.close();
    if (ledBOn){
      digitalWrite(LED_B, LOW);
      ledBOn = false;
    } else {
      digitalWrite(LED_B, HIGH);
      ledBOn = true;
    }
  }


  delay(updateTime);
  
}



