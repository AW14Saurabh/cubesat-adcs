//Mark Yeo; mark.yeo@student.unsw.edu.au
//Last Modified 2017-02-13
//Reaction Wheel System v1 - Simple Detumble v1
//This program uses a reaction wheel to eliminate any spin on the main platform, using a gyroscope sensor
//The gyro is calibrated upon startup (slow red blink x5), so hold it still until the green led lights up
//If the red led stays lit, there's probably a wiring problem (sensor not detected)

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_L3GD20_U.h>

Adafruit_L3GD20_Unified gyro = Adafruit_L3GD20_Unified(20);

float biasZ = 0;
int motorPinA = 5;
int motorPinB = 6;
int ledRed = 4;
int ledGreen = 7;

void setup(void){
  pinMode(motorPinA, OUTPUT);
  pinMode(motorPinB, OUTPUT);
  pinMode(ledRed, OUTPUT);
  pinMode(ledGreen, OUTPUT);
  digitalWrite(motorPinA, LOW);
  digitalWrite(motorPinB, LOW);
  digitalWrite(ledRed, LOW);
  digitalWrite(ledGreen, LOW);

  
  // Init sensor
  gyro.enableAutoRange(true);
  if(!gyro.begin()){
    // No gyro detected - wiring fault probably
    digitalWrite(ledRed, HIGH);
    while(1);
  }

  // Calibrate Gyro - calculates bias by averaging of a bunch of measurements while stationary
  int calibIters = 100;
  int calibTime = 10;
  int i = 0;
  for (i = 0; i < calibIters; i++){
    sensors_event_t event; 
    gyro.getEvent(&event);
    biasZ += event.gyro.z;
    //Blink red LED slowly while calibrating
    if (i % 10 == 0){
        if ((i/10) % 2 == 0){
          digitalWrite(ledRed, LOW);
        } else {
          digitalWrite(ledRed, HIGH);
        }
    }
    delay(calibTime);
  }
  biasZ /= calibIters;

  // All Ready!
  digitalWrite(ledRed, LOW);
  digitalWrite(ledGreen, HIGH);
}





float zpos = 0;                     //angular position
int i = 0;

void loop(void){
  // Poll sensor for angular velocity around z axis
  sensors_event_t event; 
  gyro.getEvent(&event);
  float wZ = event.gyro.z - biasZ;  //remove bias from measurement
  
  // Adjust speed of wheel based on current angular velocity
  float k = 2.0;                    //proportional to strength of motor response; found by trial & error - want to stop asap but not oscillate
  float motorSpeed = k * wZ;        //proportional controller
  float cappedMotorSpeed = min(1.0, abs(motorSpeed)); //cap speed at 100%
  int motorOut = int(255 * cappedMotorSpeed); //255 = 5V with analogWrite()

  // Update motor speed
  if (motorSpeed < 0){
    analogWrite(motorPinA, motorOut);
    digitalWrite(motorPinB, LOW);
  } else {
    digitalWrite(motorPinA, LOW);
    analogWrite(motorPinB, motorOut);
  }
  
  // Blink red LED to indicate controller activity
  if (i % 2 == 0){
    digitalWrite(ledRed, LOW);
  } else {
    digitalWrite(ledRed, HIGH);
  }
  i++;

  // Wait a little bit before repeating process again
  int updateTime = 10;                //ms
  delay(updateTime);
}
