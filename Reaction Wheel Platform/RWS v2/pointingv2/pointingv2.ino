// 100ms loop
// recursively printing Z axis rotation of gyroscope

#include <Wire.h>
#include <FaBo9Axis_MPU9250.h>
#define PERIOD 0.1 //100ms
#define CALIB_INTERVAL 2000   //timeout interval 2000ms
#define VR 5
#define ZF 7
#define TARGET_ANGLE 0
#define LPF_Size 20

float bias;
float gz_actual;
float angle;
float integration_sum;
float error;
float error_integrated;
float KD, KP, KI;
int motor_init = 0;

FaBo9Axis fabo_9axis;

typedef struct {
  float values[LPF_Size];
  uint8_t p;
} values;

float LPFilter(float inp);
values LPf;


void setup() {
  delay(2000);
  Serial.begin(9600);
  Serial.println("RESET");
  Serial.println();

  Serial.println("configuring device.");

  if (fabo_9axis.begin()) {
    Serial.println("configured FaBo 9Axis I2C Brick");
  } else {
    while (1){
      Serial.println("device error");
    }
  }
  
  bias = 0;
  timedOut = false; // Allow timer to fire
  angle = 0; //Set initial angle is 0
  error_integrated = 0;
  KP = KD = KI = 1.0;

  for (uint8_t i = 0; i < 20; i++)
    LPf.values[i] = 0;
  LPf.p = 0;
  
  pinMode(ZF, OUTPUT);
  pinMode(VR, OUTPUT);





  //Calculate the bias
  boolean timedOut = false; // set to true when timer fired
  double sum = 0;
  int i = 0;
  int timer = millis(); // start timer
  while(timedOut == false){
    float gx,gy,gz;
    fabo_9axis.readGyroXYZ(&gx,&gy,&gz);
    sum += gz; 
    i++;    
    if (millis() - timer >= CALIB_INTERVAL) {
      timedOut = true;
    }
    delay(10);
  }
  bias = sum/i;
  Serial.print("bias: ");
  Serial.println(bias); 
}


void loop() {
  float output;


    

  fabo_9axis.readGyroXYZ(&gx,&gy,&gz);
  gz_actual = gz - bias;
  //gz_actual = LPfilter(gz_actual);      Low Pass Filter
  //Serial.print(" gz calibrated: ");
  //Serial.println(gz_actual);

  angle += 10 * PERIOD * gz_actual;
  Serial.print(" angle: ");
  Serial.println(angle);
  //if(angle == TARGET_ANGLE)  angle = 0; //Reset once target is reached

  //change angle but not error
  error =  TARGET_ANGLE - angle;
  /*while (angle > 180){
    angle -= 360;
    //error -=TARGET_ANGLE; 
  }
  while (angle <= - 180) angle += 360;
  //while (error > 180)  error -= 360;
  while (error <= - 180) error += 360;*/

  KD = 0.89;
  KP = -0.0064;
  KI = -0.000003;
  
  //PID controller
  error_integrated += error * 10 * PERIOD;
  Serial.print(" error: "); Serial.print(error);Serial.print("\t");
  Serial.print(" error integration:  "); Serial.print(error_integrated); Serial.print("\t");
  Serial.print(" error derivative:  "); Serial.print(gz_actual); Serial.print("\t");
  output = KP * error + KI * error_integrated + KD * gz_actual;
  Serial.print(" output:  "); Serial.println(output); 

  float motorOut = 0;
  if (output > 0)
    motorOut = (output > 127.5) ? 127.5 : output;
  else
    motorOut = (output < -127.5) ? -127.5 : output;

  if (motor_init < 50000*PERIOD){
    motorOut = 0;
    motor_init++;
  }
  
  digitalWrite(ZF, LOW);
  analogWrite(VR, 127.5 + motorOut);
  Serial.write("motor output: "); Serial.print(motorOut); Serial.write("\n");   
  delay(10*PERIOD); //unit is in millisecond
}


float LPfilter(float inp){
  LPf.values[LPf.p] = inp;
  LPf.p = (LPf.p == LPF_Size) ? 0 : LPf.p+1;
  
  float avg = 0;
  uint8_t i;
  for (uint8_t i = 0; i < LPF_Size; i++);
    avg += LPf.values[i]/LPF_Size;

  return avg;
}

