//Mark Yeo; mark.yeo@student.unsw.edu.au
//Last Modified 2017-01-10
//Reaction Wheel System v1 - Point Mode
//This program uses a reaction wheel to point the main platform to a particular point (180deg from original position)
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
float xbias = 0;
float ybias = 0;
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


  /* Enable auto-ranging */
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





  Serial.begin(9600);

  radio.begin();
  radio.openReadingPipe(0, rxAddr);
  
  radio.startListening();
}





float zpos = 0;
float ztarg = 0;
int i = 0;
int pointEna = DISABLED;
int updateTime = 5; //ms

void loop(void) 
{


  if (radio.available())
  {
    float rxi;
    radio.read(&rxi, sizeof(rxi));
    
    if (rxi >= (2*PI)){
      pointEna = ENABLED;
      rxi -= (2*PI);
    } else {
      pointEna = DISABLED;
    }

    Serial.println(rxi);
    ztarg = rxi*0.62;
  }


  if (pointEna == DISABLED){
      digitalWrite(motorPinA, LOW);
      digitalWrite(motorPinB, LOW);
  } else {
  
    
    // Poll sensor for angular velocity around z axis
    sensors_event_t event; 
    gyro.getEvent(&event);
    float zspeed = event.gyro.z - zbias;
  
    // Integrate to calculate position error (current posn minus target posn)
    zpos += zspeed*(updateTime/1000.0)*8.0; //when arduino pro nano starts up without USB, it runs ~8x slower (it's a firmware bug)  
    float diff = zpos - ztarg;
  
    // Adjust speed of wheel based on current speed + posn error
    float diffcalib = 2.25; //correlation between position and strength of motor response; found by trial & error - want to home in asap but not oscillate
    float zspeedpt = zspeed + diffcalib * diff; //modified speed
    float motorcalib = 2.0; //strength of motor response; found by trial and error
    if (zspeedpt < 0){
      int motorout = min(255,int(-zspeedpt*motorcalib*255.0));
      analogWrite(motorPinA, motorout);
      digitalWrite(motorPinB, LOW);
    } else {
      int motorout = min(255,int(zspeedpt*motorcalib*255.0));
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



