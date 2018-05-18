//Mark Yeo; mark.yeo@student.unsw.edu.au
//Last Modified 2018-04-26
//Program for ctrl box v2 (TX)
// - idles for 1s after power on
// - calibrates gyro for 1s (keep box still ~5s after power on)
// - calculates orientation via dead-reckoning from gyro
// - sends azimuth (deg, init. 180deg) + state of switches over radio (ctrl box v1 format (structs)
// (note that ctrl-box RGB LED has a common +ve lead
//   so LEDs turn on when relevant pin is pulled LOW)

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_L3GD20_U.h>
#include <MahonyAHRS.h>
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
#define SAMPLE_TIME 160 //ms (based off processing time between subsequent loops)


struct packet{
  char h;
  int d;
};

RF24 radio(NRF_CE, NRF_CS);
const byte rxAddr[6] = "00001";
Adafruit_L3GD20_Unified gyro = Adafruit_L3GD20_Unified(20); //assign unique ID
Mahony filter;
struct packet tx;
int updateTime = 50; //ms
float angleTarg = 0;
int mode = DETUMBLE;
int enableState = DISABLED;
unsigned long previousMillis = 0;
//int dt = 0; //FOR CALCULATING SAMPLE_TIME
//float roll, pitch;
float yaw;
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

  //Serial.begin(9600);
  radio.begin();
  radio.setRetries(15, 15);
  radio.openWritingPipe(rxAddr);
  radio.stopListening();
  gyro.enableAutoRange(true);
  if(!gyro.begin()){
    //Serial.println("Ooops, no L3GD20 detected ... Check your wiring!");
    while(1);
  }
  filter.setSampleTime(SAMPLE_TIME); //s
  delay(500);
  calibrateGyro();
  
  // Ready!
}




void loop(void){
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= SAMPLE_TIME) {
    //dt = currentMillis - previousMillis; //FOR CALCULATING SAMPLE_TIME
    previousMillis = currentMillis;

    // Poll sensor for angular velocity around z axis
    sensors_event_t event; 
    gyro.getEvent(&event);
  
    // Update the Mahony filter, with scaled gyroscope
    filter.updateGyro(event.gyro.x - biasX, event.gyro.y - biasY, event.gyro.z - biasZ);
    //roll = filter.getRoll();
    //pitch = filter.getPitch();
    yaw = filter.getYaw();


    //print out attitude
    /*
    Serial.print(maxDt);
    Serial.print(",");
    Serial.print(dt);
    Serial.print(",");
    Serial.println(yaw);
    */
    //Serial.print(",");
    //Serial.print(pitch);
    //Serial.print(",");
    //Serial.println(roll);
    
    //read switches
    int enableState;
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

    //send attitude + switch states over RF
    tx.h = 'e';
    tx.d = enableState;
    radio.write(&tx, sizeof(tx));
    if (enableState == ENABLED){
      tx.h = 'a';
      tx.d = yaw;
      //tx.d = dt; //FOR CALCULATING SAMPLE_TIME
      radio.write(&tx, sizeof(tx));
      tx.h = 'm';
      tx.d = mode;
      radio.write(&tx, sizeof(tx));
    }
  }

  
  if (digitalRead(PB) == HIGH){
    //calibrateGyro(); to calibrate just reset arduino
    filter.resetQuat();
  }
}



