// Program for ADCS Hub
// Gathers data from sensors, calculates attitude + logs to uSD card
// Sensors:
// - L3GD20H (Gyro)
// - QMC5883 (Compass)
// Author: Mark Yeo
// Last Modified 2018-04-26

// Sensors are polled on average every xx //TO CALCULATE 26-32ms. Max poll delay <70ms (v. occasional)
// Green LED:   Data is being logged to SD card
// Yellow LED:  Data is not being logged to SD card
// Data is in format:
//      "MILLIS DT g:GYROX,GYROY,GYROZ r:PITCH,ROLL,YAW"
// e.g. "//EXAMPLE






#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_L3GD20_U.h>
#include <MechaQMC5883.h>
#include <RTClib.h>
#include <SPI.h>
#include <SD.h>
#include <MahonyAHRS.h>


#define LED_G 6
#define LED_Y 9
#define PB    5
#define SD_CS 14
#define FILE_NAME "dataAH.txt"
#define SAMPLE_TIME 30 //ms (based off processing time between subsequent loops) //100ms w/ serial print




Adafruit_L3GD20_Unified gyro = Adafruit_L3GD20_Unified(20); //assign unique ID
MechaQMC5883 qmc;
File dataFile;
Mahony filter;
float biasX = 0;
float biasY = 0;
float biasZ = 0;
float posZ = 0;

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




void setup() {
  //=================================
  // INITIALISATION
  //=================================
  pinMode(LED_G, OUTPUT);
  pinMode(LED_Y, OUTPUT);
  pinMode(SD_CS, OUTPUT);
  pinMode(PB, INPUT);

  digitalWrite(LED_G, LOW);
  digitalWrite(LED_Y, HIGH);

  //Serial.begin(9600);
  Wire.begin();

  gyro.begin(); // Initialise L3GD20H (Gyro)
  gyro.enableAutoRange(true);
  calibrateGyro();
  qmc.init();   // Initialise QMC5883 (Compass)
  qmc.setMode(Mode_Continuous,ODR_100Hz,RNG_2G,OSR_256);
  SD.begin(SD_CS);
  filter.setSampleTime(SAMPLE_TIME); //s

  //=================================
  // STARTUP ROUTINE
  //=================================
  dataFile = SD.open(FILE_NAME, FILE_WRITE);
  if(dataFile){
    dataFile.println();
    dataFile.println("Sensor Board v2 Startup ");
    dataFile.println(millis());
    dataFile.close();
  } else {
    return;
  }
  
  digitalWrite(LED_Y, LOW);
  digitalWrite(LED_G, HIGH);
}



unsigned long previousMillis = 0;
int maxDt = 0;


void loop() {
  unsigned long currentMillis = millis();
  int dt = currentMillis - previousMillis;
  if (dt >= SAMPLE_TIME) {
  //=================================
  // POLL SENSORS
  //=================================
    previousMillis = currentMillis;
    sensors_event_t eventG; 
    gyro.getEvent(&eventG);
    float wx = eventG.gyro.x;
    float wy = eventG.gyro.y;
    float wz = eventG.gyro.z;
    uint16_t mx,my,mz;
    qmc.read(&mx,&my,&mz);


  


    float roll, pitch, yaw;
  
    // Update the Mahony filter, with scaled gyroscope
    filter.updateGyro(eventG.gyro.x - biasX, eventG.gyro.y - biasY, eventG.gyro.z - biasZ);
  
    roll = filter.getRoll();
    pitch = filter.getPitch();
    yaw = filter.getYaw();


    
  
  
    //=================================
    // DATA FORMATTING
    //=================================
    if (dt > maxDt && dt < SAMPLE_TIME + 50){ //for debugging kinda
      maxDt = dt;
    }
    char strT[25];
    sprintf(strT, "%lu %d %d ", currentMillis, dt, maxDt);
    String strW = "g:" + String(wx,4) + "," + String(wy,4) + "," + String(wz,4) + " ";
    char strM[20];
    sprintf(strM, "m:%4d,%4d,%4d ", mx, my, mz);
    String strR = "r:" + String(roll,3) + "," + String(pitch,3) + "," + String(yaw,3) + " ";

  
    //=================================
    // DATA OUTPUT
    //=================================
  
    //Output to Serial when button is pressed (arduino sometimes bugs out w/ uploading when Serial is constantly in use)
    /*
    if (digitalRead(PB) == LOW){
      Serial.print(strT);
      Serial.print(strW);
      Serial.print(strM);
      Serial.print(strR);
      Serial.println();
    }
    */
    //Write to SD card
    dataFile = SD.open(FILE_NAME, FILE_WRITE);
    if(dataFile){
      dataFile.print(strT);
      dataFile.print(strW);
      dataFile.print(strM);
      dataFile.print(strR);
      dataFile.println();
      dataFile.close();
      digitalWrite(LED_G, HIGH);
      digitalWrite(LED_Y, LOW);
    } else {
      digitalWrite(LED_G, LOW);
      digitalWrite(LED_Y, HIGH);
    }
  }
  if (digitalRead(PB) == HIGH){
    calibrateGyro();
    filter.resetQuat();
  }
}

