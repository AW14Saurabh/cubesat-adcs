//Mark Yeo; mark.yeo@student.unsw.edu.au
//Last Modified 2018-04-25
//To set + check RTC time

#include <Wire.h>
#include <RTClib.h>

DS3231 rtc; //if garbage values (e.g. "20@5-25-@5  @5:@5:85"), make sure Wire.begin(); is run + no other devices at 0x68


void setup(void){
  Serial.begin(9600);
  Wire.begin();

  rtc.begin();  // Initialise DS3231  (Real Time Clock)
  rtc.adjust(DateTime(__DATE__, __TIME__)); //use this to set RTC time
}



void loop(void){
  //comment out loop code when setting time to minimise compile/upload time
  DateTime now = rtc.now();
  char buf[25];
  strncpy(buf,"YYYY-MM-DD hh:mm:ss \0",25);
  Serial.println(now.format(buf));
  delay(500);
  
}



