// Author: Mark Yeo
// Last Modified 2018-05-18
// Program for ADCS Hub Arduino 1 (linked to Gyro + Magnetometer)
// Gathers data from a L3GD20H gyro using SoftI2C Library + keeps track of orientation by dead reckoning
// Yellow LED toggles each time the gyro is polled; push button resets orientation to [0,0,180]

//Note: The L3GD20H gyro library has been modified to work with SoftI2C.
//Side note: Arduino 1 on the ADCS Hub is also hooked up to a QMC5883 compass (cheap Chinese version of the HMC5883).
//  RWS doesn't actually need it but it's there for future use - if you want to use it you'll probably
//  need to modify a QMC5883 library to work with SoftI2C . Btw HMC5883 libraries don't work w/ the
//  QMC5883 out of the box.


#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_L3GD20_U_SOFTI2C.h>


#define ARDUINO_GYRO  1
#define MIN_SAMPLE_TIME 30  //sets the minimum amount of time to wait between measurements
#define LED_Y 6
#define PB    5


Adafruit_L3GD20_Unified gyro = Adafruit_L3GD20_Unified(20); //assign unique ID

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
int dt = 0;




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


union u_byte_float {
 byte b[28];
 float val[7];
} uAtt;
  //wx wy wz q0 q1 q2 q3;


// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
void requestEvent() {
  uAtt.val[0] = wx;
  uAtt.val[1] = wy;
  uAtt.val[2] = wz;
  uAtt.val[3] = q0;
  uAtt.val[4] = q1;
  uAtt.val[5] = q2;
  uAtt.val[6] = q3;
  int i;
  for (i=0; i<28; i++){
    Wire.write(uAtt.b[i]);
  }
}


void setup() {
  //Serial.begin(9600);
  Wire.begin(ARDUINO_GYRO);                // join i2c bus with address #8
  Wire.onRequest(requestEvent); // register event
  pinMode(LED_Y, OUTPUT);
  digitalWrite(LED_Y, HIGH);

  gyro.enableAutoRange(true);
  gyro.begin(); // Initialise L3GD20H (Gyro)

  calibrateGyro();

  initHeading();
  digitalWrite(LED_Y, LOW); 
}



unsigned long previousMillis = 0;
bool ledOn = false;


void loop() {
  unsigned long currentMillis = millis();
  dt = currentMillis - previousMillis;
  if (dt >= MIN_SAMPLE_TIME) {
    previousMillis = currentMillis;
    sensors_event_t eventG; 
    gyro.getEvent(&eventG);
    wx = eventG.gyro.x - biasX;
    wy = eventG.gyro.y - biasY;
    wz = eventG.gyro.z - biasZ;


    updateHeading(wx, wy, wz, dt);
  
    rollDeg = getRoll();
    pitchDeg = getPitch();
    yawDeg = getYaw();
/*
    char strT[25];
    sprintf(strT, "%lu %d ", currentMillis, dt);
    String strW = "g:" + String(wx,4) + "," + String(wy,4) + "," + String(wz,4) + " ";
    String strR = "r:" + String(rollDeg,3) + "," + String(pitchDeg,3) + "," + String(yawDeg,3) + " ";

    Serial.print(strT);
    Serial.print(strW);
    Serial.print(strR);
    Serial.println();
*/
    if (ledOn){
      digitalWrite(LED_Y, LOW);
      ledOn = false;
    } else {
      digitalWrite(LED_Y, HIGH);
      ledOn = true;
    }


  }
  if (digitalRead(PB) == HIGH){
    initHeading();
  }
}


