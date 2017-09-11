//Program to detumble/point Reaction Wheel System v2 using Control Box
//Mark Yeo; mark.yeo@student.unsw.edu.au
//Last Modified 2017-09-12

//Libraries
#include <Wire.h>
#include <FaBo9Axis_MPU9250.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <PID_v1.h>
//Pins
#define RED_LED 8
#define GRN_LED 9
#define MTR_VR 5
#define MTR_ZF 7
//States
#define DISABLED 0
#define ENABLED 1
#define DETUMBLE 0
#define POINT 1
#define CCW 0
#define CW 1

/*
//Function Definitions
void setupIO();
void setupMPU9250();
void calibGyro();
void setupRadio();
void setupPID();
bool getPacket(struct BoxCmd *box);
float getWz();
int dAngle(float wz);
void driveMotor(int mtrVel);
*/
//Just a bunch of structs
struct Bias{
  float x;
  float y;
  float z;
};
struct Packet{
  char h;
  int d;
};
struct BoxCmd{
  bool mtrEna;
  bool mode;
  int angle;
};

//Setup + Global Variables
FaBo9Axis fabo_9axis;
RF24 radio(19,18);
const byte rxAddr[6] = "00001";
double g_PIDSetpoint, g_PIDInput, g_PIDOutput;  //PID variables
double g_dKp=2, g_dKi=0, g_dKd=0;  //PID tuning constants for detumbling
double g_pKp=2, g_pKi=0, g_pKd=0;  //PID tuning constants for pointing
PID PIDCtrl(&g_PIDInput, &g_PIDOutput, &g_PIDSetpoint, g_dKp, g_dKi, g_dKd, DIRECT);
struct Bias g_gyroBias;
unsigned long g_prevPollTime = 0;
unsigned long g_prevCmdTime = 0;
int g_currAngle = 0;
int g_cycleTime = 100;



void setup(){
  Serial.begin(9600);
  setupIO();
  digitalWrite(RED_LED, HIGH);
  setupMPU9250();
  calibGyro();
  setupRadio();
  setupPID();
  digitalWrite(RED_LED, LOW);
  g_prevPollTime = millis();
  g_prevCmdTime = millis();
}



void loop(){
  //=== Get data from gyro ===
  float wz = getWz(); //Poll gyro
  g_currAngle += dAngle(wz);  //Calculate current angle using basic integration

  //=== Get commands from controller box ===
  struct BoxCmd box;
  bool boxConnected = true;
  if (getPacket(&box)){
    digitalWrite(GRN_LED, HIGH);
    g_prevCmdTime = millis();
  } else {
    if (millis() - g_prevCmdTime > 2000){ //if 2s since last message received from box
      digitalWrite(GRN_LED, LOW);
      boxConnected = false;
      Serial.println("Lost connection to box, disabling motor");
    }
  }

  //=== Calculate + output speed to motor ===
  int mtrVel = 0;
  if (boxConnected && box.mtrEna == ENABLED){
    if (box.mode == DETUMBLE){
      g_PIDInput = wz;
      g_PIDSetpoint = 0;
      PIDCtrl.SetTunings(g_dKp, g_dKi, g_dKd);
    } else {// i.e. box.mode == POINT
      g_PIDInput = g_currAngle;
      g_PIDSetpoint = box.angle;
      PIDCtrl.SetTunings(g_pKp, g_pKi, g_pKd);
    }
    PIDCtrl.Compute();
    mtrVel = g_PIDOutput;
  }
  driveMotor(mtrVel);
  
  delay(g_cycleTime);
}






//=== Setup Functions ===
void setupIO(){
  pinMode(RED_LED, OUTPUT);
  pinMode(GRN_LED, OUTPUT);
  pinMode(MTR_VR, OUTPUT);
  pinMode(MTR_ZF, OUTPUT);
  digitalWrite(RED_LED, LOW);
  digitalWrite(GRN_LED, LOW);
  digitalWrite(MTR_VR, LOW);
  digitalWrite(MTR_ZF, LOW);
}

void setupMPU9250(){
  fabo_9axis.begin();
  fabo_9axis.configMPU9250(MPU9250_GFS_1000, MPU9250_AFS_2G); //MPU9250_GFS_250/500/1000(~3rev/s)/2000 (max deg/s)
}

void calibGyro(){
  int numSamples = 100;
  int calibTime = 2000; //approx time to calibrate (ms)
  int i;
  struct Bias cumuBias;
  cumuBias.x = 0;
  cumuBias.y = 0;
  cumuBias.z = 0;
  for (i = 0; i < numSamples; i++){
    float gx,gy,gz;
    fabo_9axis.readGyroXYZ(&gx,&gy,&gz);
    cumuBias.x += gx;
    cumuBias.y += gy;
    cumuBias.z += gz;
    if (i % 10 == 0){
        if ((i/10) % 2 == 0){
          digitalWrite(RED_LED, LOW);
        } else {
          digitalWrite(RED_LED, HIGH);
        }
    }
    delay(calibTime/numSamples);
  }
  g_gyroBias.x = cumuBias.x / numSamples;
  g_gyroBias.y = cumuBias.y / numSamples;
  g_gyroBias.z = cumuBias.z / numSamples;
  digitalWrite(RED_LED, HIGH);
}

void setupRadio(){
  radio.begin();
  radio.openReadingPipe(0, rxAddr);  
  radio.startListening();
}

void setupPID(){
  g_PIDInput = 0;
  g_PIDSetpoint = 0;
  PIDCtrl.SetMode(AUTOMATIC); //turn the PID on
  PIDCtrl.SetOutputLimits(-255, 255); //clamps the output to -255 to 255
  PIDCtrl.SetSampleTime(g_cycleTime);
  PIDCtrl.SetControllerDirection(REVERSE);
}

//=== Loop Functions ===
bool getPacket(struct BoxCmd *box){
  bool rxFlag = false;
  while (radio.available()){
    struct Packet packet;
    radio.read(&packet, sizeof(packet));
    rxFlag = true;
    if (packet.h == 'a'){
      box->angle = packet.d;
      
    } else if (packet.h == 'm'){
      if (packet.d == 0){
        box->mode = DETUMBLE;
      } else if (packet.d == 1) {
        box->mode = POINT;
      }
      
    } else if (packet.h == 'e'){
      if (packet.d == 0){
        box->mtrEna = DISABLED;
      } else if (packet.d == 1) {
        box->mtrEna = ENABLED;
      }
    }
  }
  return rxFlag;
}

float getWz(){
  float gx,gy,gz;
  fabo_9axis.readGyroXYZ(&gx,&gy,&gz);
  gz -= g_gyroBias.z;
  return gz;
}

int dAngle(float wz){
  unsigned long currPollTime = millis();
  int changeInAngle = int(wz*(currPollTime-g_prevPollTime)/1000.0); //does weird stuff if RHS is left as a float - probably a result of some weird float precision stuff
  g_prevPollTime = currPollTime;
  return changeInAngle;
}

void driveMotor(int mtrVel){
  //Convert mtrVel (-255 to 255) to mtrSpd (0-255) & mtrDir (CCW/CW)
  int mtrDir;
  int mtrSpd;
  if (mtrVel >= 0){
    mtrDir = CCW; //+ve rotation around z axis
    mtrSpd = mtrVel;
  } else {
    mtrDir = CW;
    mtrSpd = -mtrVel;
  }
  //In case the given mtrVel was too big
  if (mtrSpd > 255){
    mtrSpd = 255;
  }
  //Write out mtrSpd + mtrDir to motor driver
  if (mtrDir == CCW){
    digitalWrite(MTR_ZF, LOW); //debug radiospinwheel, change if wrong way around
  } else {
    digitalWrite(MTR_ZF, HIGH);
  }
  analogWrite(MTR_VR, mtrSpd);
}


