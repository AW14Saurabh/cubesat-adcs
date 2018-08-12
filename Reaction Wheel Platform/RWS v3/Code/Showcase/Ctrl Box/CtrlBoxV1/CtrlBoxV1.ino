//Mark Yeo; mark.yeo@student.unsw.edu.au
//Last Modified 2018-08-10
//Program for ctrl box v1 (the one with the pot)
// - sends state of switches + angle of pot over radio coded within a single int

#include <Wire.h>
#include <RF24.h>

#define POT_ANG A0
#define SW_ENA  A2
#define SW_MODE  A1
#define NRF_CE 9  //CE is pin 9 on Ctrl Box v1
#define NRF_CS 10 //CSN is pin 10 on Ctrl Box v1
#define DETUMBLE 0
#define POINT 1
#define DISABLED 0
#define ENABLED 1
#define MIN_TX_TIME 150 //minimum delay time between every transmission




RF24 radio(NRF_CE, NRF_CS);
const byte rxAddr[6] = "00062";

int updateTime = 50; //ms
float angleTarg = 0;
int mode = DETUMBLE;
int enableState = DISABLED;
float angleOut = 0;



void setup(void){
  pinMode(SW_ENA, INPUT);
  pinMode(SW_MODE, INPUT);

  Serial.begin(9600);
  radio.begin();
  radio.setRetries(15, 15);
  radio.openWritingPipe(rxAddr);
  radio.stopListening();
}


unsigned long previousTxMillis = 0;

void loop(void){
  unsigned long currentMillis = millis();
  
  if (currentMillis - previousTxMillis >= MIN_TX_TIME) {
    previousTxMillis = currentMillis;


    if (digitalRead(SW_ENA) == HIGH){
      enableState = ENABLED;
      
      if (digitalRead(SW_MODE) == HIGH){
        mode = POINT;
      } else {
        mode = DETUMBLE;
      }
      
      float sensorIn = analogRead(POT_ANG);
      float sensorCalib = (sensorIn-50)/780.0; //pot output calibrated between 0 & 1
      //Serial.println(sensorCalib);
      angleOut = sensorCalib * 2*PI; //values range between 0-2PI
      angleOut = 2*PI - angleOut; //angle measured around z+ axis (CCW)
      angleOut = constrain(angleOut, 0, 2*PI);
    } else {
      enableState = DISABLED;
    }
    
    // angleOut is a float in radians between 0-2PI, enableState & mode are either 0 or 1
    unsigned int message = constrain(angleOut*16383.0/(2*PI),0,16383);
    message += enableState *  ((unsigned int)B10000000*256 + B00000000);
    message += mode *         (B01000000*256 + B00000000);

    
    radio.write(&message, sizeof(message));
    
    
    float angleOutDeg = angleOut/PI*180;
    Serial.print("Tx: ");
    Serial.print(enableState);
    Serial.print(mode);
    Serial.print(" ");
    Serial.print(angleOutDeg);
    Serial.print(" ");
    Serial.println(message);
    
    
  }

  
}



