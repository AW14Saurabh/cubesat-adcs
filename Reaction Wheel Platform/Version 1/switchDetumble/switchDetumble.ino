//Mark Yeo; mark.yeo@student.unsw.edu.au
//Last Modified 2017-01-10
//Reaction Wheel System v1 - Detumble Mode
//This program uses a reaction wheel to eliminate any spin on the main platform, using a gyroscope sensor
//The gyro is calibrated upon startup (slow red blink x5), so hold it still until the green led lights up
//If the red led stays lit, there's probably a wiring problem (sensor not detected)

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_L3GD20_U.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#define ENABLED 1
#define DISABLED 0


Adafruit_L3GD20_Unified gyro = Adafruit_L3GD20_Unified(20);
float zbias = 0;
int motorPinA = 5;
int motorPinB = 6;
int ledRed = 4;
int ledGreen = 7;
RF24 radio(18, 19);
const byte rxAddr[6] = "00001";




void setup(void) 
{
  pinMode(motorPinA, OUTPUT);
  pinMode(motorPinB, OUTPUT);
  pinMode(ledRed, OUTPUT);
  pinMode(ledGreen, OUTPUT);
  digitalWrite(motorPinA, LOW);
  digitalWrite(motorPinB, LOW);
  digitalWrite(ledRed, LOW);
  digitalWrite(ledGreen, LOW);

  //dunno what this is but it's in here?
  gyro.enableAutoRange(true);
  
  // Init sensor
  if(!gyro.begin())
  {
    // No gyro detected - wiring fault probably
    digitalWrite(ledRed, HIGH);
    while(1);
  }




  // Calibrate Gyro
  int calibIters = 100;
  int calibTime = 10;
  int i = 0;
  for (i = 0; i < calibIters; i++){
    sensors_event_t event; 
    gyro.getEvent(&event);
    zbias += event.gyro.z;
    if (i % 10 == 0){
        if ((i/10) % 2 == 0){
          digitalWrite(ledRed, LOW);
        } else {
          digitalWrite(ledRed, HIGH);
        }
    }
    delay(calibTime);
  }
  zbias /= calibIters;

  // Ready!
  digitalWrite(ledRed, LOW);
  digitalWrite(ledGreen, HIGH);



  radio.begin();
  radio.openReadingPipe(0, rxAddr);
  
  radio.startListening();

  
  



}





float zpos = 0;
int i = 0;
int detumbleEna = DISABLED;
int updateTime = 10; //ms


void loop(void) 
{
  if (radio.available())
  {
    float rxi;
    radio.read(&rxi, sizeof(rxi));
    
    if (rxi >= (2*PI)){
      detumbleEna = ENABLED;
      rxi -= (2*PI);
    } else {
      detumbleEna = DISABLED;
    }
  }
  
  if (detumbleEna == DISABLED){
      digitalWrite(motorPinA, LOW);
      digitalWrite(motorPinB, LOW);
  } else {

  
  
  
    
    // Poll sensor for angular velocity around z axis
    sensors_event_t event; 
    gyro.getEvent(&event);
    float zspeed = event.gyro.z - zbias;
    
    // Adjust speed of wheel based on current angular velocity
    float motorcalib = 2.0; //strength of motor response; found by trial & error - want to stop asap but not oscillate
    if (zspeed < 0){
      int motorout = min(255,int(-zspeed*motorcalib*255.0));
      analogWrite(motorPinA, motorout);
      digitalWrite(motorPinB, LOW);
    } else {
      int motorout = min(255,int(zspeed*motorcalib*255.0));
      digitalWrite(motorPinA, LOW);
      analogWrite(motorPinB, motorout);
    }
    
    // Blink red LED to indicate controller activity
    if (i % 2 == 0){
      digitalWrite(ledRed, LOW);
    } else {
      digitalWrite(ledRed, HIGH);
    }
    i++;
  }
  
  delay(updateTime);
}
