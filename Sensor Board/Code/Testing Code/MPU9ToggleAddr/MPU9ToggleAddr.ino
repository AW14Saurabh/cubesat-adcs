#include <Wire.h>
#include <FaBo9Axis_MPU9250.h>

#define MPU6_CS 8
#define MPU9_CS 9


FaBo9Axis fabo_9axis;
void setup() {
  pinMode(MPU6_CS, OUTPUT);
  pinMode(MPU9_CS, OUTPUT);
  digitalWrite(MPU9_CS, HIGH);
  digitalWrite(MPU6_CS, HIGH);

  Serial.begin(9600);  
  Serial.println("Begin Sensor Setup");
  
  digitalWrite(MPU6_CS, LOW);
  if (fabo_9axis.begin()) {
    Serial.println("MPU9250 online");
  } else {
    Serial.println("MPU9250 device error");
    while(1);
  }
  digitalWrite(MPU6_CS, HIGH);
}

int flag = 1;


void loop() {
  float ax,ay,az;
  float gx,gy,gz;
  float mx,my,mz;
  float temp;

  

  if(flag == 1){
    digitalWrite(MPU9_CS, HIGH);
    digitalWrite(MPU6_CS, LOW);
    fabo_9axis.setMPU(0x69);
    flag = 0;
  }else{
    digitalWrite(MPU9_CS,LOW);
    digitalWrite(MPU6_CS, HIGH);
    fabo_9axis.setMPU(0x68);
    flag = 1;  
  }
  
  Serial.println(fabo_9axis.getMPU());
  
  fabo_9axis.readAccelXYZ(&ax,&ay,&az);
  fabo_9axis.readGyroXYZ(&gx,&gy,&gz);
  fabo_9axis.readMagnetXYZ(&mx,&my,&mz);
  fabo_9axis.readTemperature(&temp);

  String mpu9accel = "ax: " + String(ax,3) + " ay: " + String(ay,3) + " az: " + String(az,3);
  String mpu9gyro = "gx: " + String(gx,3) + " gy: " + String(gy,3) + " gz: " + String(gz,3);
  String mpu9mag = "mx: " + String(mx,3) + " my: " + String(my,3) + " mz: " + String(mz,3);
  String mpu9temp = "temp: " + String(temp,3);

  Serial.println(mpu9accel);
  Serial.println(mpu9gyro);
  Serial.println(mpu9mag);
  Serial.println(mpu9temp);

  delay(100);
}
