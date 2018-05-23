// Author: Mark Yeo
// Last Modified 2018-05-21
// Program for RWS Board Arduino. Does the following:
// - updates internal variables when data sent from pwr arduino:
//  - satellite heading (q0,q1,q2,q3)
//  - satellite w (x,y,z)
//  - ctrl box settings (motorEnable, mode, targetAngle)
// - if motors enabled + in detumble mode, runs detumble controller to calculate required combined dH to be generated by all 4 wheels
// - if motors enabled + in point mode, runs point controller to calculate required combined dH
// - once required dH is calculated, distributes dH to each wheel using the pseudoinverse method
//    then scales back wheel dW's based on software limits to keep physical motor behaviour from entering non-linear region
// - sends motor speed data to pwr arduino when requested


#include <Wire.h>

// I2C communications to master
#define ARDUINO_RWS   3
#define NUM_GYRO_RX_FLOATS 7
#define NUM_COMMS_RX_UINTS 1
#define NUM_RWS_TX_INTS 4
#define SIZE_FLOAT 4
#define SIZE_UINT 2
#define SIZE_INT 2
#define SQRT3 1.732
#define SQRT2 1.414
#define MOTOR_MAX_ACCEL 248.8 //rad/s
#define WHEEL_I 1.41E-05
#define MAX_MOTOR_W 1376 //rad/s (measured)
#define MOTOR_MIDDLE_SPEED 754 //rad/s (non-zero minimum speed)

// Motor interface pins
#define MOT1_SPD 10
#define MOT1_DIR 12
#define MOT1_FREQ A2
#define MOT2_SPD 11
#define MOT2_DIR 13
#define MOT2_FREQ A3
#define MOT3_SPD 9
#define MOT3_DIR 8
#define MOT3_FREQ A1
#define MOT4_SPD 3
#define MOT4_DIR 4
#define MOT4_FREQ A0

#define LED_G 5
#define LED_B 6
#define PUSHBUTTON 7


// States
#define DISABLED 0
#define ENABLED 1
#define DETUMBLE 0
#define POINT 1


float satW[3] = {0,0,0};
float q0 = 0;
float q1 = 0;
float q2 = 0;
float q3 = 0;
float motorW[4] = {0,0,0,0};
int freqIn[4] = {0,0,0,0};
float angleTarg = 0;
int mode = DETUMBLE;
int enableState = DISABLED;

// Unions for I2C comms
union uByteSevenFloat {
 byte b[NUM_GYRO_RX_FLOATS*SIZE_FLOAT];
 float val[NUM_GYRO_RX_FLOATS];
} uGyroRx;

union uByteOneUInt {
 byte b[NUM_COMMS_RX_UINTS*SIZE_UINT];
 unsigned int val;
} uCommsRx;

union uByteFourInt {
 byte b[NUM_RWS_TX_INTS*SIZE_INT];
 int val[NUM_RWS_TX_INTS];
} uRwsTx;



// Functions for I2C communications

void receiveEvent(int howMany) {
  int i;
  byte data[100]; //big array to store input bytes
  i = 0;
  while (Wire.available()) {
    data[i] = Wire.read();
    i = i + 1;
  }
  for (i=0; i<NUM_GYRO_RX_FLOATS*SIZE_FLOAT; i++){
    uGyroRx.b[i] = data[i];
  }
  int offset = i;
  for (i=0; i<NUM_COMMS_RX_UINTS*SIZE_UINT; i++){
    uCommsRx.b[i] = data[offset+i];
  }
  satW[0] = uGyroRx.val[0];
  satW[1] = uGyroRx.val[1];
  satW[2] = uGyroRx.val[2];
  q0 = uGyroRx.val[3];
  q1 = uGyroRx.val[4];
  q2 = uGyroRx.val[5];
  q3 = uGyroRx.val[6];
  //process comms message (wrapped up in a single unsigned int)
  unsigned int message = uCommsRx.val;
  enableState = message >> 15;
  message = message & ((unsigned int)B01111111*256 + B11111111);
  mode = message >> 14;
  message = message & ((unsigned int)B10111111*256 + B11111111);    
  angleTarg = constrain(message*2*PI/16383.0,0,2*PI);


  //TEMPORARY COMMS BYPASS
  //enableState = ENABLED;
  //mode = POINT;
  //angleTarg = -PI/2;
}

void requestEvent() {
  uRwsTx.val[0] = freqIn[0];
  uRwsTx.val[1] = freqIn[1];
  uRwsTx.val[2] = freqIn[2];
  uRwsTx.val[3] = freqIn[3];
  int i;
  for (i=0; i<NUM_RWS_TX_INTS*SIZE_INT; i++){
    Wire.write(uRwsTx.b[i]);
  }
}



void detumbleController(float reqWheelCombDH[], int dtMillis){
    //simple P controller
    float error[3];
    int i;
    float P = -0.05;  //calibrated for air bearing
    //float P = -0.003;  //for stairwell suspension test
    
    for (i=0; i<3; i++){
      error[i] = 0 - satW[i]; //target w is [0,0,0]
      reqWheelCombDH[i] = P*error[i];
      reqWheelCombDH[i] *= (float)dtMillis/1000.0;  //normalise for dt (assumes the following cycle
                                // will have a similar dt to the previous cycle
      //Serial.print(reqWheelCombDH[i]);
      //Serial.print(" ");
    }
    
}

void calcEul(float theta[]){
  theta[0] = atan2f(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2);
  theta[1] = asinf(-2.0f * (q1*q3 - q0*q2));
  theta[2] = atan2f(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3);
}

void pointController(float reqWheelCombDH[], int dtMillis){
  int i;
  float theta[3];
  calcEul(theta);
  int axis = 0; //0 = roll, 1 = pitch, 2 = yaw
  //angleTarg = 0;
  float error = angleTarg - theta[axis];


  //take shortest path (ignores current w)
  while (error > PI){
    error -= 2*PI;
  }
  while (error <= -PI){
    error += 2*PI;
  }
  
  Serial.print(angleTarg);
  Serial.print(" ");
  Serial.println(theta[axis]);
  float P = -0.01;  //calibrated for air bearing
  float D = 0.05;   //calibrated for air bearing
  //float P = -0.0002;  //for stairwell suspension test //-0.001 for z
  //float D = 0.002;   //for stairwell suspension test //0.003 for z
  float axisDH = P*error + D*satW[axis];
  axisDH *= (float)dtMillis/1000.0;
  for (i=0; i<3; i++){
    reqWheelCombDH[i] = 0;
  }
  reqWheelCombDH[axis] = axisDH;
  reqWheelCombDH[axis+1] = axisDH; //for rotation about +x+y axis
  reqWheelCombDH[axis+2] = axisDH; //+x+y+z (untested)
}


void calculateWheelWs(float reqWheelCombDH[], int dtMillis){
  float x = reqWheelCombDH[0];
  float y = reqWheelCombDH[1];
  float z = reqWheelCombDH[2];
  float wheelDH[4];
  
  wheelDH[0] = SQRT3/4.0*(0*x + SQRT2*y - z);
  wheelDH[1] = SQRT3/4.0*(-SQRT2*x + 0 - z);
  wheelDH[2] = SQRT3/4.0*(0*x + -SQRT2*y - z);
  wheelDH[3] = SQRT3/4.0*(SQRT2*x + 0 - z);


  //convert h to w
  int i;
  float motorDW[4];
  for (i=0; i<4; i++){
    motorDW[i] = wheelDH[i]/WHEEL_I;
  }
  
  
  //scale down dW if accel is too large
  float maxMotorDW = 0;
  for (i=0; i<4; i++){
    if (abs(motorDW[i]) > maxMotorDW){
      maxMotorDW = abs(motorDW[i]);
    }
  }
  float dt = (float)dtMillis/1000.0;
  float maxAccel = MOTOR_MAX_ACCEL*dt;
  if (maxMotorDW > maxAccel){
    for (i=0; i<4; i++){
      motorDW[i] *= maxAccel/maxMotorDW;
    }
  }


  //if any motor is saturated, no change to any motor
  float maxMotorW = 0;
  for (i=0; i<4; i++){
    float nextMotorW = (float)motorW[i] + motorDW[i];
    if (abs(nextMotorW) > maxMotorW){
      maxMotorW = abs(nextMotorW);
    }
  }
  if (maxMotorW > MAX_MOTOR_W){
    for (i=0; i<4; i++){
      motorDW[i] = 0;
    }
  }

  //update motorW
  for (i=0; i<4; i++){
    motorW[i] += motorDW[i];
  }

  
  //if pushbutton is pressed, reset motors to midpoint operation range
  int pbVal = MOTOR_MIDDLE_SPEED;
  if (digitalRead(PUSHBUTTON) == HIGH){
    for (i=0; i<4; i++){
      motorW[i] = pbVal;
    }
  }
/*
  Serial.print("mW:");
  for (i=0; i<4; i++){
    Serial.print(motorW[i]);
    Serial.print(" ");
  }*/
}




void motorOut(){



  int out[4];
  int i;
  for (i=0; i<4; i++){
    out[i] = 203-(int)(motorW[i]*203.0/MAX_MOTOR_W);
    out[i] = constrain(out[i], 0, 203);
  }

  
  analogWrite(MOT1_SPD, out[0]);
  analogWrite(MOT2_SPD, out[1]);
  analogWrite(MOT3_SPD, out[2]);
  analogWrite(MOT4_SPD, out[3]);
  /*
  Serial.print("out:");
  for (i=0; i<4; i++){
    Serial.print(out[i]);
    Serial.print(" ");
  }
  */
  
}


void motorDisable(){
  //set motor speeds to 0
  analogWrite(MOT1_SPD, 255);
  analogWrite(MOT2_SPD, 255);
  analogWrite(MOT3_SPD, 255);
  analogWrite(MOT4_SPD, 255);
}

void motorReset(){
  //reset motors to midpoint operation range
  int pbVal = MOTOR_MIDDLE_SPEED;
  int i;
  for (i=0; i<4; i++){
    motorW[i] = pbVal;
  }
}


void setup() {
  Wire.begin(ARDUINO_RWS);                // join i2c bus with address #8
  Wire.onReceive(receiveEvent); // register event
  Wire.onRequest(requestEvent); // register event
  Serial.begin(9600);           // start serial for output
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  pinMode(PUSHBUTTON, INPUT);

  pinMode(MOT1_SPD, OUTPUT);
  pinMode(MOT1_DIR, OUTPUT);
  pinMode(MOT1_FREQ, INPUT);
  pinMode(MOT2_SPD, OUTPUT);
  pinMode(MOT2_DIR, OUTPUT);
  pinMode(MOT2_FREQ, INPUT);
  pinMode(MOT3_SPD, OUTPUT);
  pinMode(MOT3_DIR, OUTPUT);
  pinMode(MOT3_FREQ, INPUT);
  pinMode(MOT4_SPD, OUTPUT);
  pinMode(MOT4_DIR, OUTPUT);
  pinMode(MOT4_FREQ, INPUT);

  digitalWrite(MOT1_DIR,LOW);
  digitalWrite(MOT2_DIR,LOW);
  digitalWrite(MOT3_DIR,LOW);
  digitalWrite(MOT4_DIR,LOW);
  digitalWrite(MOT1_SPD,HIGH);
  digitalWrite(MOT2_SPD,HIGH);
  digitalWrite(MOT3_SPD,HIGH);
  digitalWrite(MOT4_SPD,HIGH);
  
}



unsigned long prevmillis = 0;

void loop() {
  unsigned long currmillis = millis();
  
  
  float reqWheelCombDH[3] = {0,0,0};
  int dtMillis = currmillis - prevmillis;
  prevmillis = currmillis;
  if (mode == DETUMBLE){
    detumbleController(reqWheelCombDH, dtMillis);
    digitalWrite(LED_G, HIGH);
    digitalWrite(LED_B, LOW);
  } else if (mode == POINT){
    pointController(reqWheelCombDH, dtMillis);
    digitalWrite(LED_B, HIGH);
    digitalWrite(LED_G, LOW);
  }
  calculateWheelWs(reqWheelCombDH, dtMillis);
  if (enableState == ENABLED){
    motorOut();
  } else {
    //motorDisable();
    motorReset();
    motorOut();
    digitalWrite(LED_G, LOW);
    digitalWrite(LED_B, LOW);
  }
  
  
  
  
  freqIn[0] = analogRead(MOT1_FREQ);
  freqIn[1] = analogRead(MOT2_FREQ);
  freqIn[2] = analogRead(MOT3_FREQ);
  freqIn[3] = analogRead(MOT4_FREQ);

  
  
  //Debug printfs
  /*
  String strC = "c:" + String(angleTarg,4) + "," + String(mode) + "," + String(enableState) + " ";
  String strW = "w:" + String(satW[0],4) + "," + String(satW[1],4) + "," + String(satW[2],4) + " ";
  String strQ = "q:" + String(q0,6) + "," + String(q1,6) + "," + String(q2,6) + " " + String(q3,6) + " ";
  Serial.print(currmillis);
  Serial.print(" ");
  Serial.print(strC);
  Serial.print(strW);
  Serial.print(strQ);
  */
  Serial.println();

  delay(10);
}




