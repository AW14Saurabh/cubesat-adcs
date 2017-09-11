//Mark Yeo; mark.yeo@student.unsw.edu.au
//Last Modified 2017-02-14
//Reaction Wheel System v1 - Detumble or Point Mode
//This program uses a reaction wheel to either detumble or point the main platform to a particular point
// based on a remote switch position and potentiometer
//The gyro is calibrated upon startup (slow red blink x5), so hold it still until the green led lights up
//If the red led stays lit, there's probably a wiring problem (sensor not detected)


#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_L3GD20_U.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#define DISABLED 0
#define ENABLED 1
#define MOTOR_PIN_A 5
#define MOTOR_PIN_B 6
#define RED_LED 4
#define GREEN_LED 7
#define DETUMBLE 0
#define POINT 1


Adafruit_L3GD20_Unified gyro = Adafruit_L3GD20_Unified(20);
float xbias = 0;
float ybias = 0;
float biasZ = 0;

RF24 radio(18, 19);
const byte rxAddr[6] = "00001";



void setup(void){
  //Set up pins
  pinMode(MOTOR_PIN_A, OUTPUT);
  pinMode(MOTOR_PIN_B, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  digitalWrite(MOTOR_PIN_A, LOW);
  digitalWrite(MOTOR_PIN_B, LOW);
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, LOW);

  // Initialise Gyro
  gyro.enableAutoRange(true);
  if(!gyro.begin()){
    // No gyro detected - wiring fault probably
    digitalWrite(RED_LED, HIGH);
    while(1);
  }
  
  // Calibrate Gyro
  int calibIters = 100;
  int calibTime = 80;
  int i = 0;
  for (i = 0; i < calibIters; i++){
    sensors_event_t event; 
    gyro.getEvent(&event);
    biasZ += event.gyro.z;
    if (i % 10 == 0){
        if ((i/10) % 2 == 0){
          digitalWrite(RED_LED, LOW);
        } else {
          digitalWrite(RED_LED, HIGH);
        }
    }
    delay(calibTime);
  }
  biasZ /= calibIters;

  //Initialise Radio
  radio.begin();
  radio.openReadingPipe(0, rxAddr);  
  radio.startListening();

  Serial.begin(9600);

  // Ready!
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, HIGH);
}



struct packet{
  char h;
  int d;
};

float angle = 0;
float angleTarg = 0;
int i = 0;
int motorEna = DISABLED;
int updateTime = 40; //ms
int mode = DETUMBLE;

void loop(void){

  //Check for any radio messages
  if (radio.available()){
    struct packet rx;
    radio.read(&rx, sizeof(rx));

    if (rx.h == 'a'){
      angleTarg = (2*PI) - (rx.d * 2*PI / 360.0);
      Serial.println(angleTarg);
    } else if (rx.h == 'm'){
      mode = rx.d;
    } else if (rx.h == 'e'){
      motorEna = rx.d;
    }
    i = 0; //radio 'watchdog' reset - see below
  }

  //If remote switch is in 'Disabled' position, turn off motor
  if (motorEna == DISABLED || i > 25){    //radio 'watchdog' - if reaction wheel misses 10 radio messages in a row,
                                          // disables motor until radio contact reestablished. Control box spams messages
                                          // at 10Hz, so 100ms/msg / (5ms/loop * 8ms/ms) * 10msg = 25loops
      digitalWrite(MOTOR_PIN_A, LOW);
      digitalWrite(MOTOR_PIN_B, LOW);
      digitalWrite(RED_LED, LOW);

  } else {    
    // Poll gyro
    sensors_event_t event; 
    gyro.getEvent(&event);
    float wZ = event.gyro.z - biasZ;      //wZ = angular velocity around z axis
    float motorSpeed = 0;
    
    if (mode == DETUMBLE){
      // Adjust speed of wheel based on current angular velocity
      float kP = 2.0;                    //proportional to strength of motor response; found by trial & error - want to stop asap but not oscillate
      motorSpeed = kP * wZ;        //proportional controller

    } else if (mode == POINT){
      // Integrate to calculate angle error (current angle minus target angle)
      angle += wZ*(updateTime/1000.0);  //when arduino pro nano starts up without USB, it runs ~8x slower (it's a firmware bug)  
      float angleErr = angle - angleTarg;
      
      // Proportional Derivative Controller
      float kP = 3.5;// 4,1.5                       //correlation between angle and strength of motor response; found by trial & error - want to home in asap but not oscillate
      float kD = 1.3;                       //corr. between speed and motor response - a smaller value results in quicker homing but also oscillations
      motorSpeed = kP * angleErr + kD * wZ;         //PD controller
    }

    
    // Drive motor
    float cappedMotorSpeed = min(1.0, abs(motorSpeed)); //cap speed at 100%
    int motorOut = (int)(255 * cappedMotorSpeed); //255 = 5V with analogWrite()
    if (motorSpeed < 0){
      analogWrite(MOTOR_PIN_A, motorOut);
      digitalWrite(MOTOR_PIN_B, LOW);
    } else {
      digitalWrite(MOTOR_PIN_A, LOW);
      analogWrite(MOTOR_PIN_B, motorOut);
    }
    
    // Blink red LED to indicate controller activity
    if (i % 2 == 0){
      digitalWrite(RED_LED, LOW);
    } else {
      digitalWrite(RED_LED, HIGH);
    }
    i++;
  }
  
  delay(updateTime);
}



