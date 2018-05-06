// Program for ADCS Hub
// Gathers data from sensors using SoftI2C Library
// Sensors:
// - L3GD20H (Gyro)
// - QMC5883 (Compass)
// Author: Mark Yeo
// Last Modified 2018-05-06

//Note: compass doesn't work, not sure why


#include <Wire.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_L3GD20_U.h>
#include <MechaQMC5883.h>

#define MIN_SAMPLE_TIME 30  //sets the minimum amount of time to wait between measurements



Adafruit_L3GD20_Unified gyro = Adafruit_L3GD20_Unified(20); //assign unique ID
MechaQMC5883 qmc;

float wx = 0;
float wy = 0;
float wz = 0;
float q0, q1, q2, q3;
float biasX = 0;
float biasY = 0;
float biasZ = 0;
char anglesComputed = 0;
float roll = 0;
float pitch = 0;
float yaw = 0;
char messageOut[60];
int dt = 0;

void initHeading(){
  q0 = 1.0f;
  q1 = 0.0f;
  q2 = 0.0f;
  q3 = 0.0f;
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
  roll = atan2f(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2);
  pitch = asinf(-2.0f * (q1*q3 - q0*q2));
  yaw = atan2f(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3);
  anglesComputed = 1;
}

float getRoll() {
  if (!anglesComputed) computeAngles();
  return roll * 57.29578f;
}
float getPitch() {
  if (!anglesComputed) computeAngles();
  return pitch * 57.29578f;
}
float getYaw() {
  if (!anglesComputed) computeAngles();
  return yaw * 57.29578f + 180.0f;
}
float getRollRadians() {
  if (!anglesComputed) computeAngles();
  return roll;
}
float getPitchRadians() {
  if (!anglesComputed) computeAngles();
  return pitch;
}
float getYawRadians() {
  if (!anglesComputed) computeAngles();
  return yaw;
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


union u_tag {
 byte b[4];
 float fval;
} u;

// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
void requestEvent() {
  /*Wire.write(dt); //2 bytes
  Wire.write(wx); //4 bytes
  Wire.write(wy); //4 bytes
  Wire.write(wz); //4 bytes
  Wire.write(roll); //4 bytes
  Wire.write(pitch); //4 bytes
  */
  u.fval = yaw;
  int i;
  for (i=0; i<4; i++){
    Wire.write(u.b[i]); //4 bytes
  }
}


void setup() {
  Serial.begin(9600);
  Wire.begin(8);                // join i2c bus with address #8
  Wire.onRequest(requestEvent); // register event

  gyro.begin(); // Initialise L3GD20H (Gyro)
  gyro.enableAutoRange(true);
  calibrateGyro();
  qmc.init();   // Initialise QMC5883 (Compass)
  qmc.setMode(Mode_Continuous,ODR_100Hz,RNG_2G,OSR_256);

  initHeading();
}



unsigned long previousMillis = 0;



void loop() {
  unsigned long currentMillis = millis();
  dt = currentMillis - previousMillis;
  if (dt >= MIN_SAMPLE_TIME) {
    previousMillis = currentMillis;
    sensors_event_t eventG; 
    gyro.getEvent(&eventG);
    wx = eventG.gyro.x;
    wy = eventG.gyro.y;
    wz = eventG.gyro.z;
    //uint16_t mx,my,mz;
    //qmc.read(&mx,&my,&mz);


    float roll, pitch, yaw;
    updateHeading(eventG.gyro.x - biasX, eventG.gyro.y - biasY, eventG.gyro.z - biasZ, dt);
  
    roll = getRoll(); //TO DO: these don't actually get sent b/c roll pitch yaw are global variables, the ones here are local
    pitch = getPitch();
    yaw = getYaw();

    char strT[25];
    sprintf(strT, "%lu %d ", currentMillis, dt);
    String strW = "g:" + String(wx,4) + "," + String(wy,4) + "," + String(wz,4) + " ";
    //char strM[20];
    //sprintf(strM, "m:%4d,%4d,%4d ", mx, my, mz);
    String strR = "r:" + String(roll,3) + "," + String(pitch,3) + "," + String(yaw,3) + " ";

    Serial.print(strT);
    Serial.print(strW);
    //Serial.print(strM);
    Serial.print(strR);
    Serial.println();
  }
}

//dt wx wy wz roll pitch yaw;
//50 chars: "60 g:-0.0006,0.0009,0.0154 r:0.100,-0.369,179.804 "

