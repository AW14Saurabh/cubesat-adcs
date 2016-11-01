#include <Wire.h>
#include <LSM303.h>


LSM303 compass;
float prevhd = 0;
int motorout = 0;
void setup() {
  Serial.begin(9600);
  Wire.begin();
  compass.init();
  compass.enableDefault();
  //Calibration
  compass.m_min = (LSM303::vector<int16_t>){-12462, -3420, 12351};
  compass.m_max = (LSM303::vector<int16_t>){-4602, +3746, +19327};
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);

}

void loop() {
  compass.read();
  float hd = compass.heading();
  
  if (hd >= 180 && hd-prevhd >= 0){
    motorout += 20;
  }
  if (hd < 180 && hd-prevhd < 0){
    motorout -= 20;
  }
  prevhd = hd;

  if (motorout > 255){
    motorout = 255;
  }
  if (motorout < -255){
    motorout = -255;
  }
  if (motorout >= 0){
    analogWrite(5, motorout);
    digitalWrite(6, LOW);
  } else {
    digitalWrite(5, LOW);
    analogWrite(6, -motorout);
  }
  Serial.println(hd);
  delay(20);
}
