// 100ms loop
// recursively printing Z axis rotation of gyroscope

#include <Wire.h>
#include <FaBo9Axis_MPU9250.h>
#define PERIOD 0.1 //100ms
#define KP 1
#define KI 1
#define KD 1
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

FaBo9Axis fabo_9axis;

void setup() {
  Serial.begin(9600);
  Serial.println("RESET");
  Serial.println();

  Serial.println("configuring device.");

  if (fabo_9axis.begin()) {
    Serial.println("configured FaBo 9Axis I2C Brick");
  } else {
    Serial.println("device error");
    while(1);
  }
  
  bias = 0;
  timedOut = false; // Allow timer to fire
  angle = 0; //Set initial angle is 0
  error_integrated = 0;
}

void loop() {
  float sum = 0;
  float gx,gy,gz;
  int i = 0;
  float output;
  

  //Calculate the bias
  while(timedOut == false){
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
  }
  
  fabo_9axis.readGyroXYZ(&gx,&gy,&gz);
  gz_actual = gz - bias;
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
  

  //PID controller
  error_integrated += error * PERIOD;
  Serial.print(" error: "); Serial.print(error);Serial.print("\t");
  Serial.print(" error integration:  "); Serial.print(error_integrated); Serial.print("\t");
  Serial.print(" error derivative:  "); Serial.print(-gz_actual); Serial.print("\t");
  output = KP * error + KI * error_integrated + KD * (-gz_actual);
  Serial.print(" output:  "); Serial.println(output); 
 
  delay(1000*PERIOD); //unit is in millisecond
}




