//Program to detumble Reaction Wheel System v2
//Mark Yeo; mark.yeo@student.unsw.edu.au
//Last Modified 11Sept'17

#include <Wire.h>
#include <FaBo9Axis_MPU9250.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <PID_v1.h>

#define RED_LED 8
#define GRN_LED 9
#define MTR_VR 5
#define MTR_ZF 7

#define DISABLED 0
#define ENABLED 1
#define DETUMBLE 0
#define POINT 1
#define CCW 0
#define CW 1

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
struct Bias g_gyroBias;
int g_prevPollTime = 0;
int g_currAngle = 0;
int g_prevCmdTime = 0;






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

void setup(){
  Serial.begin(9600);
  setupIO();
  digitalWrite(RED_LED, HIGH);
  setupMPU9250();
  calibGyro();
  setupRadio();
  digitalWrite(RED_LED, LOW);
  g_prevPollTime = millis();
  g_prevCmdTime = millis();
}














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
  int currPollTime = millis();
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

void loop(){
  //Loop Parameters
  int cycleTime = 100;
  
  
  float wz = getWz(); //Poll gyro
  g_currAngle += dAngle(wz);  //Calculate current angle using basic integration
  
  struct BoxCmd box;
  bool boxConnected = true;
  if (getPacket(&box)){
    digitalWrite(GRN_LED, HIGH);
    g_prevCmdTime = millis();
  } else {
    if (millis() - g_prevCmdTime > 3000){ //if 3s since last message received from box (doesn't like 2000 for some reason - turns off immediately)
      digitalWrite(GRN_LED, LOW);
      boxConnected = false;
      Serial.println("Lost connection to box, disabling motor");
    }
  }
  int mtrVel = 0;
  if (boxConnected && box.mtrEna == ENABLED){
    if (box.mode == DETUMBLE){
      mtrVel = -30;
    } else if (box.mode == POINT){
      mtrVel = 60;
    }
  }
  driveMotor(mtrVel);

  
  
  
  





  //Serial.print(gz);
  //Serial.print(",");
  //Serial.println(g_currAngle);

  //Serial.print("Angle: ");
  //Serial.print(box.angle);
  //Serial.print(" Mode: ");
  //Serial.print(box.mode);
  //Serial.print(" Enable: ");
  //Serial.println(box.enable);

  
  
  
  delay(cycleTime);
}

