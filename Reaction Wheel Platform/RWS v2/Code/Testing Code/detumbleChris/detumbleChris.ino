
 // 100ms loop
// recursively printing Z axis rotation of gyroscope

#include <Wire.h>
#include <FaBo9Axis_MPU9250.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_L3GD20_U.h>
#define PERIOD 0.05 //100ms
#define KP 0.5
#define KI 0.005
#define KD 30
#define MOTOR_PIN_A 5
#define MOTOR_PIN_B 6
#define TARGET_ANGLE 180

unsigned long timer; // the timer
boolean timedOut; // set to true when timer fired
unsigned long INTERVAL = 2000; // the timeout interval
float bias;
float gz_actual;
float angle;
float integration_sum;
float error;
float error_integrated;

int ZF = 7;
//int VR = 5;

//float KD = -1.0;

FaBo9Axis fabo_9axis;
Adafruit_L3GD20_Unified gyro = Adafruit_L3GD20_Unified(20);
float xbias = 0;
float ybias = 0;
float biasZ = 0;


void setup() {
  Serial.begin(9600);
  Serial.println("RESET");
  Serial.println();

  Serial.println("configuring device.");

  /*if (fabo_9axis.begin()) {
    Serial.println("configured FaBo 9Axis I2C Brick");
  } else {
    Serial.println("device error");
    while(1);
  }*/
  // Initialise Gyro
  gyro.enableAutoRange(true);
  if(!gyro.begin()){
    // No gyro detected - wiring fault probably
    //digitalWrite(RED_LED, HIGH);
    Serial.write("No gyro detected\n");
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
          //digitalWrite(RED_LED, LOW);
        } else {
          //digitalWrite(RED_LED, HIGH);
        }
    }
    delay(calibTime);
  }
  biasZ /= calibIters;
  
  bias = 0;
  timedOut = false; // Allow timer to fire
  angle = 0; //Set initial angle is 0
  error_integrated = 0;

  pinMode(ZF, OUTPUT);
//  pinMode(VR, OUTPUT);
  pinMode(MOTOR_PIN_A, OUTPUT);
  pinMode(MOTOR_PIN_B, OUTPUT);
}

void loop() {
  float sum = 0;
  float gx,gy,gz;
  int i = 0;
  float output;
  

  //Calculate the bias
  /*while(timedOut == false){
    timer = millis();// start timer
    //Serial.print("Time: ");
    //Serial.println(timer);
    
    if (timer == INTERVAL) {
    // timed out'
      timedOut = true; // don't do this again
      // you can reset the single shot timer by setting
      //  timedOut = false;
      //  timer = millis();
      Serial.println("Stopping calculating bias");
      
      bias = sum/i;
      Serial.print(" bias: ");
      Serial.println(bias); 
      break;
    }
    
    fabo_9axis.readGyroXYZ(&gx,&gy,&gz);
    sum += gz; 
    i += 1;
  }*/
  
  //fabo_9axis.readGyroXYZ(&gx,&gy,&gz);
  sensors_event_t event; 
  gyro.getEvent(&event);
  gz_actual = event.gyro.z - biasZ; 
  //gz_actual = gz - bias;
  //Serial.print(" gz calibrated: ");
  //Serial.println(gz_actual);

  angle += PERIOD * gz_actual;
  Serial.print(" angle: ");
  Serial.println(angle);
  //if(angle == TARGET_ANGLE)  angle = 0; //Reset once target is reached

  //change angle but not error
  error =  TARGET_ANGLE - angle;
  if(angle > 180){
    angle -= 360;
    //error -=TARGET_ANGLE;
    
  }
  if(angle <= - 180) angle += 360;

  if(error > 180)  error -= 360;
  if(error <= - 180) error += 360;
  

 /*if (-gz_actual > 0)
    KD = -1.1;
  else
    KD = -2.55;*/

  //PID controller
  error_integrated += error * PERIOD;
  Serial.print(" error: "); Serial.print(error);Serial.print("\t");
  Serial.print(" error integration:  "); Serial.print(error_integrated); Serial.print("\t");
  Serial.print(" error derivative:  "); Serial.print(-gz_actual); Serial.print("\t");
  output = /*KP * error + KI * error_integrated +*/ KD*(-gz_actual);
  Serial.print(" output:  "); Serial.println(output); 


  /*if (output > 127.5)
    output = 127.5;
  else if (output < -127.5)
    output = -127.5;
  digitalWrite(ZF, LOW);  
  analogWrite(VR, output + 127.5);
  Serial.print(" output v: "); Serial.println(output);*/

  float cappedMotorSpeed = min(255.0, abs(output)); //cap speed at 100%
    int motorOut = (int)(cappedMotorSpeed); //255 = 5V with analogWrite()
    if (output < 0){
      analogWrite(MOTOR_PIN_A, motorOut);
      digitalWrite(MOTOR_PIN_B, LOW);
    } else {
      digitalWrite(MOTOR_PIN_A, LOW);
      analogWrite(MOTOR_PIN_B, motorOut);
    }
  
 delay(1000*PERIOD); //unit is in millisecond
}




