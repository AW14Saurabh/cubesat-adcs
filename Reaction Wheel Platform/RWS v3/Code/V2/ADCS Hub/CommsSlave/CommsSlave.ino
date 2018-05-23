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

#define ARDUINO_COMMS  2
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

#define NUM_GYRO_RX_FLOATS 7
#define NUM_RWS_RX_INTS 4
#define NUM_PWR_RX_INTS 2
#define NUM_COMMS_TX_UINTS 1
#define SIZE_FLOAT 4
#define SIZE_UINT 2
#define SIZE_INT 2
#define TRANSMISSION_0 0
#define TRANSMISSION_1 1


RF24 radio(NRF_CE, NRF_CS);
const byte rxAddr[6] = "00062";
File dataFile;
unsigned int messageIn = 0;
float wx = 0;
float wy = 0;
float wz = 0;
float q0 = 0;
float q1 = 0;
float q2 = 0;
float q3 = 0;
int w1 = 0;
int w2 = 0;
int w3 = 0;
int w4 = 0;
int current = 0;
int voltage = 0;




union uByteSevenFloat {
 byte b[NUM_GYRO_RX_FLOATS*SIZE_FLOAT];
 float val[NUM_GYRO_RX_FLOATS];
} uGyroRx;

union uByteFourInt {
 byte b[NUM_RWS_RX_INTS*SIZE_INT];
 int val[NUM_RWS_RX_INTS];
} uRwsRx;

union uByteTwoInt {
  byte b[NUM_PWR_RX_INTS*SIZE_INT];
  int val[NUM_PWR_RX_INTS];
} uPwrRx;

union uByteOneUInt {
 byte b[NUM_COMMS_TX_UINTS*SIZE_UINT];
 unsigned int val;
} uCommsTx;






void requestEvent() {
  uCommsTx.val = messageIn;
  int i;
  for (i=0; i<2; i++){
    Wire.write(uCommsTx.b[i]); //2 bytes
  }
}

void receiveEvent(int bytesIn) {
  int i;
  byte data[100]; //big array to store input bytes
  int offset;

  i = 0;
  while (Wire.available()) {
    data[i] = Wire.read();
    i = i + 1;
  }
  
  if (data[0] == TRANSMISSION_0){
    offset = 1;
    for (i=0; i<NUM_GYRO_RX_FLOATS*SIZE_FLOAT; i++){
      uGyroRx.b[i] = data[i + offset];
    }
    wx = uGyroRx.val[0];
    wy = uGyroRx.val[1];
    wz = uGyroRx.val[2];
    q0 = uGyroRx.val[3];
    q1 = uGyroRx.val[4];
    q2 = uGyroRx.val[5];
    q3 = uGyroRx.val[6];
    
  } else if (data[0] == TRANSMISSION_1){
    offset = 1;
    for (i=0; i<NUM_RWS_RX_INTS*SIZE_INT; i++){
      uRwsRx.b[i] = data[offset+i];
    }
    offset += i;
    for (i=0; i<NUM_PWR_RX_INTS*SIZE_INT; i++){
      uPwrRx.b[i] = data[offset+i];
    }
    w1 = uRwsRx.val[0];
    w2 = uRwsRx.val[1];
    w3 = uRwsRx.val[2];
    w4 = uRwsRx.val[3];
    
    current = uPwrRx.val[0];
    voltage = uPwrRx.val[1];
  }
  
  
  

  
}


int startmillis;

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
  
  //Serial.begin(9600);
  Wire.begin(ARDUINO_COMMS);
  Wire.onRequest(requestEvent); // register event
  Wire.onReceive(receiveEvent); // register event
  
  radio.begin();
  radio.openReadingPipe(0, rxAddr);  
  radio.startListening();

  
  SD.begin(SD_CS);

  
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
  startmillis = millis();
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

/*
    //TEMPORARY RADIO BYPASS
    mode = DETUMBLE;
    if (millis()-startmillis < 5000){
      enableState = DISABLED;
    } else {
      enableState = ENABLED;
    }
    angleTarg = 0;
    messageIn = constrain((angleTarg+PI)*16383.0/(2*PI),0,16383);
    messageIn += enableState *  ((unsigned int)B10000000*256 + B00000000);
    messageIn += mode *         (B01000000*256 + B00000000);
*/

    unsigned int message = messageIn;
    enableState = message >> 15;
    message = message & ((unsigned int)B01111111*256 + B11111111);
    mode = message >> 14;
    message = message & ((unsigned int)B10111111*256 + B11111111);    
    angleTarg = constrain(message*2*PI/16383.0,0,2*PI);

    //toggle Green LED when radio message received
    if (ledGOn){
      digitalWrite(LED_G, LOW);
      ledGOn = false;
    } else {
      digitalWrite(LED_G, HIGH);
      ledGOn = true;
    }
  }


/*
  String strC = "c:" + String(angleTarg,4) + "," + String(mode) + "," + String(enableState) + " ";
  String strW = "w:" + String(wx,4) + "," + String(wy,4) + "," + String(wz,4) + " ";
  String strQ = "q:" + String(q0,6) + "," + String(q1,6) + "," + String(q2,6) + " " + String(q3,6) + " ";
  String strWh = "wh:" + String(w1) + "," + String(w2) + "," + String(w3) + "," + String(w4) + " ";
  String strP = "p:" + String(current) + "," + String(voltage) + " ";
  */
  /*
  Serial.print(currentMillis);
  Serial.print(" ");
  Serial.print(strC);
  Serial.print(strW);
  Serial.print(strQ);
  Serial.print(strWh);
  Serial.print(strP);
  Serial.println();
*/
  String strC = String(angleTarg,4) + "," + String(mode) + "," + String(enableState) + ",";
  String strW = String(wx,4) + "," + String(wy,4) + "," + String(wz,4) + ",";
  String strQ = String(q0,6) + "," + String(q1,6) + "," + String(q2,6) + " " + String(q3,6) + ",";
  String strWh = String(w1) + "," + String(w2) + "," + String(w3) + "," + String(w4) + ",";
  String strP = String(current) + "," + String(voltage) + ",";
  
  dataFile = SD.open(FILE_NAME, FILE_WRITE);
  if(dataFile){
    dataFile.print(currentMillis);
    dataFile.print(" ");
    dataFile.print(strC);
    dataFile.print(strW);
    dataFile.print(strQ);
    dataFile.print(strWh);
    dataFile.print(strP);
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


  //delay(updateTime);
  
}



