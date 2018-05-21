
#include <Wire.h>
#define ARDUINO_RWS   12
#define NUM_GYRO_RX_FLOATS 7
#define NUM_COMMS_RX_UINTS 1
#define NUM_RWS_TX_INTS 4
#define SIZE_FLOAT 4
#define SIZE_UINT 2
#define SIZE_INT 2

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

#define DISABLED 0
#define ENABLED 1
#define DETUMBLE 0
#define POINT 1


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
float angleTarg = 0;
int mode = DETUMBLE;
int enableState = DISABLED;


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







void setup() {
  Wire.begin(ARDUINO_RWS);                // join i2c bus with address #8
  Wire.onReceive(receiveEvent); // register event
  Wire.onRequest(requestEvent); // register event
  Serial.begin(9600);           // start serial for output
}


void loop() {
  w1 = analogRead(MOT1_FREQ);
  w2 = analogRead(MOT2_FREQ);
  w3 = analogRead(MOT3_FREQ);
  w4 = analogRead(MOT4_FREQ);
  int i;
  for (i=0; i<NUM_GYRO_RX_FLOATS; i++){
  Serial.print(uGyroRx.val[i]);
  Serial.print(" ");
  }
  Serial.print("     ");
  Serial.print(uCommsRx.val);
  Serial.println();
  
  delay(100);
}

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

  wx = uGyroRx.val[0];
  wy = uGyroRx.val[1];
  wz = uGyroRx.val[2];
  q0 = uGyroRx.val[3];
  q1 = uGyroRx.val[4];
  q2 = uGyroRx.val[5];
  q3 = uGyroRx.val[6];
  
  unsigned int message = uCommsRx.val;
  enableState = message >> 15;
  message = message & ((unsigned int)B01111111*256 + B11111111);
  mode = message >> 14;
  message = message & ((unsigned int)B10111111*256 + B11111111);    
  angleTarg = constrain(message*2*PI/16383.0,0,2*PI);
}

void requestEvent() {
  uRwsTx.val[0] = w1;
  uRwsTx.val[1] = w2;
  uRwsTx.val[2] = w3;
  uRwsTx.val[3] = w4;
  int i;
  for (i=0; i<NUM_RWS_TX_INTS*SIZE_INT; i++){
    Wire.write(uRwsTx.b[i]);
  }
}



