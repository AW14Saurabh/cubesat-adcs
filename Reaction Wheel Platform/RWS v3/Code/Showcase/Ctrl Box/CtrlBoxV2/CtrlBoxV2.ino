//Mark Yeo; mark.yeo@student.unsw.edu.au
//Last Modified 2018-04-26
//Program for ctrl box v2 (TX)
// - idles for 1s after power on
// - calibrates gyro for 1s (keep box still ~5s after power on)
// - calculates orientation via dead-reckoning from gyro
// - sends azimuth (deg, init. 180deg) + state of switches over radio (ctrl box v2 format (single int))
// (note that ctrl-box RGB LED has a common +ve lead so LEDs turn on when relevant pin is pulled LOW)

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_L3GD20_U.h>
#include <SPI.h>
#include <RF24.h>

#define LED_R 5
#define LED_G 6
#define LED_B 9
#define SW_L  4
#define SW_R  8
#define PB    7
#define NRF_CE 14
#define NRF_CS 15
#define DETUMBLE 0
#define POINT 1
#define DISABLED 0
#define ENABLED 1
#define MIN_GYRO_POLL_TIME 30  //sets the minimum amount of time to wait between measurements
#define MIN_TX_TIME 150 //transmit every 100ms




RF24 radio(NRF_CE, NRF_CS);
const byte rxAddr[6] = "00062";
Adafruit_L3GD20_Unified gyro = Adafruit_L3GD20_Unified(20); //assign unique ID

int updateTime = 50; //ms
float angleTarg = 0;
int mode = DETUMBLE;
int enableState = DISABLED;

float yaw;
float wx = 0;
float wy = 0;
float wz = 0;
float q0, q1, q2, q3;
float biasX = 0;
float biasY = 0;
float biasZ = 0;

char anglesComputed = 0;
float rollRad = 0;
float pitchRad = 0;
float yawRad = 0;
float rollDeg = 0;
float pitchDeg = 0;
float yawDeg = 0;
char messageOut[60];




void initHeading(){
  q0 = 1.0f;
  q1 = 0.0f;
  q2 = 0.0f;
  q3 = 0.0f;
  anglesComputed = 0;
}

void updateHeading(float gx, float gy, float gz, int dtms)
{
  float recipNorm;
  float qa, qb, qc;
  float dtSec = (float)dtms/1000.0;

  // Integrate rate of change of quaternion
  gx *= (0.5f * dtSec);   // pre-multiply common factors
  gy *= (0.5f * dtSec);
  gz *= (0.5f * dtSec);
  qa = q0;
  qb = q1;
  qc = q2;
  q0 += (-qb * gx - qc * gy - q3 * gz);
  q1 += (qa * gx + qc * gz - q3 * gy);
  q2 += (qa * gy - qb * gz + q3 * gx);
  q3 += (qa * gz + qb * gy - qc * gx);

  // Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
  anglesComputed = 0;
}

// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float invSqrt(float x)
{
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  y = y * (1.5f - (halfx * y * y));
  return y;
}

void computeAngles()
{
  rollRad = atan2f(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2);
  pitchRad = asinf(-2.0f * (q1*q3 - q0*q2));
  yawRad = atan2f(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3);
  anglesComputed = 1;
}

float getRoll() {
  if (!anglesComputed) computeAngles();
  return rollRad * 57.29578f;
}
float getPitch() {
  if (!anglesComputed) computeAngles();
  return pitchRad * 57.29578f;
}
float getYaw() {
  if (!anglesComputed) computeAngles();
  return yawRad * 57.29578f + 180.0f;
}
float getRollRadians() {
  if (!anglesComputed) computeAngles();
  return rollRad;
}
float getPitchRadians() {
  if (!anglesComputed) computeAngles();
  return pitchRad;
}
float getYawRadians() {
  if (!anglesComputed) computeAngles();
  return yawRad;
}



void calibrateGyro(){
  // Calibrate Gyro - calculates bias by averaging of a bunch of measurements while stationary
  int calibIters = 100;
  int calibTime = 10; //calibrates for 100*10ms = 1s
  int i = 0;
  biasX = 0;
  biasY = 0;
  biasZ = 0;
  delay(1000);
  for (i = 0; i < calibIters; i++){
    sensors_event_t event; 
    gyro.getEvent(&event);
    biasX += event.gyro.x;
    biasY += event.gyro.y;
    biasZ += event.gyro.z;
    delay(calibTime);
  }
  biasX /= calibIters;
  biasY /= calibIters;
  biasZ /= calibIters;
}




void setup(void){
  //Initialisation
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  pinMode(SW_L, INPUT);
  pinMode(SW_R, INPUT);
  pinMode(PB, INPUT);
  digitalWrite(LED_R, HIGH);
  digitalWrite(LED_G, HIGH);
  digitalWrite(LED_B, HIGH);

  Serial.begin(9600);
  radio.begin();
  radio.setRetries(15, 15);
  radio.openWritingPipe(rxAddr);
  radio.stopListening();
  gyro.enableAutoRange(true);
  if(!gyro.begin()){
    Serial.println("Ooops, no L3GD20 detected ... Check your wiring!");
    while(1);
  }
  delay(500);
  calibrateGyro();
  initHeading();
  // Ready!
}


unsigned long previousGyroMillis = 0;
unsigned long previousTxMillis = 0;
float angleOut = 0;

void loop(void){
  unsigned long currentMillis = millis();
  unsigned long gyroDt = currentMillis - previousGyroMillis;
  unsigned long txDt = currentMillis - previousTxMillis;
  if (gyroDt >= MIN_GYRO_POLL_TIME) {
    previousGyroMillis = currentMillis;

    // Poll sensor for angular velocity around z axis
    sensors_event_t eventG; 
    gyro.getEvent(&eventG);
    wx = eventG.gyro.x;
    wy = eventG.gyro.y;
    wz = eventG.gyro.z;


    updateHeading(eventG.gyro.x - biasX, eventG.gyro.y - biasY, eventG.gyro.z - biasZ, gyroDt);
    rollDeg = getRoll();
    pitchDeg = getPitch();
    yawDeg = getYaw();

    /*
    char strT[25];
    sprintf(strT, "%lu %d ", currentMillis, gyroDt);
    String strW = "g:" + String(wx,4) + "," + String(wy,4) + "," + String(wz,4) + " ";
    String strR = "r:" + String(rollDeg,3) + "," + String(pitchDeg,3) + "," + String(yawDeg,3) + " ";

    Serial.print(strT);
    Serial.print(strW);
    Serial.print(strR);
    Serial.println();
    */
    //read switches + update LEDs

    if (digitalRead(SW_L) == HIGH){
      enableState = ENABLED;
      digitalWrite(LED_R, HIGH);
      if (digitalRead(SW_R) == HIGH){
        mode = POINT;
        digitalWrite(LED_G, HIGH);
        digitalWrite(LED_B, LOW);
      } else {
        mode = DETUMBLE;
        digitalWrite(LED_G, LOW);
        digitalWrite(LED_B, HIGH);
      }
    } else {
      enableState = DISABLED;
      digitalWrite(LED_R, LOW);
      digitalWrite(LED_G, HIGH);
      digitalWrite(LED_B, HIGH);
    }
  }
  
  angleOut = yawRad; //comment out if using below code
  //angleOut = PI;
  /*
  if (Serial.available() > 0) {
    char charIn = Serial.read();
    if (charIn == 'a'){
      angleOut = 0;
    } else if (charIn == 'b'){
      angleOut = PI/2;
    } else if (charIn == 'c'){
      angleOut = PI;
    } else if (charIn == 'd'){
      angleOut = -PI/2;
    }
  }
  */
  if (txDt >= MIN_TX_TIME) {
    previousTxMillis = currentMillis;
    
    unsigned int message = constrain((angleOut+PI)*16383.0/(2*PI),0,16383);
    message += enableState *  ((unsigned int)B10000000*256 + B00000000);
    message += mode *         (B01000000*256 + B00000000);
    radio.write(&message, sizeof(message));
    
    
    
    Serial.print("Tx: ");
    Serial.print(enableState);
    Serial.print(mode);
    Serial.print(" ");
    Serial.print(yawRad);
    Serial.print(" ");
    Serial.println(message);
  }

  
  if (digitalRead(PB) == HIGH){
    //calibrateGyro(); to calibrate just reset arduino
    initHeading();
  }
}



