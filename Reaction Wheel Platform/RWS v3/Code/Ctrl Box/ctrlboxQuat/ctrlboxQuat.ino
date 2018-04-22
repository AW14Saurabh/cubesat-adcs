#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_L3GD20_U.h>
#include <MahonyAHRS.h>

#define PUSHBUTTON 7
#define SAMPLE_TIME 32 //ms (based off processing time between subsequent loops)


Adafruit_L3GD20_Unified gyro = Adafruit_L3GD20_Unified(20); //assign unique ID
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


void setup(void) 
{
  Serial.begin(9600);
  Serial.println("Gyroscope Test"); Serial.println("");
  gyro.enableAutoRange(true);
  if(!gyro.begin())
  {
    Serial.println("Ooops, no L3GD20 detected ... Check your wiring!");
    while(1);
  }
  sensor_t sensor;
  gyro.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" rad/s");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" rad/s");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" rad/s");  
  Serial.println("------------------------------------");
  Serial.println("");
  pinMode(PUSHBUTTON, INPUT);
  calibrateGyro();
  filter.setSampleTime(SAMPLE_TIME); //s
  delay(500);
}



unsigned long previousMillis = 0;
int maxDt = 0;

void loop(void) 
{
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= SAMPLE_TIME) {
    unsigned long dt = currentMillis - previousMillis;
    if (dt > maxDt && dt < 50){
      maxDt = dt;
    }
    // save the last time you blinked the LED
    previousMillis = currentMillis;

    // Poll sensor for angular velocity around z axis
    sensors_event_t event; 
    gyro.getEvent(&event);
  
    // Integrate to calculate position error (current posn minus target posn)
    //posZ += wZ*(SAMPLE_TIME/1000.0);//*8.0; //when arduino pro nano starts up without USB, it runs ~8x slower (it's a firmware bug)  
    //Serial.print(",");
    //Serial.print(posZ/3.14159*180);




    float roll, pitch, heading;
  
    // Update the Mahony filter, with scaled gyroscope
    filter.updateGyro(event.gyro.x - biasX, event.gyro.y - biasY, event.gyro.z - biasZ);
  
    roll = filter.getRoll();
    pitch = filter.getPitch();
    heading = filter.getYaw();

    Serial.print(maxDt);
    Serial.print(",");
    Serial.print(heading);
    Serial.print(",");
    Serial.print(pitch);
    Serial.print(",");
    Serial.println(roll);





    
  }
  if (digitalRead(PUSHBUTTON) == HIGH){
    calibrateGyro();
    filter.resetQuat();
  }
}

