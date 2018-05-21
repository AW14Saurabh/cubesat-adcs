

#define MOT1_SPD 10
#define MOT1_DIR 12
#define MOT1_FREQ A2
#define MOT2_SPD 11
#define MOT2_DIR 13
#define MOT2_FREQ A3
#define MOT3_SPD 9
#define MOT3_DIR 8
#define MOT3_FREQ A1
#define MOT4_SPD 3
#define MOT4_DIR 4
#define MOT4_FREQ A0
#define MOTX_SPD MOT4_SPD
#define MOTX_DIR MOT4_DIR
#define MOTX_FREQ MOT4_FREQ

int brightness = 0;    // how bright the spd is
int fadeAmount = 1;    // how many points to fade the spd by


// the setup routine runs once when you press reset:
void setup() {
  // declare pin 9 to be an output:
  pinMode(MOTX_SPD, OUTPUT);
  pinMode(MOTX_DIR, OUTPUT);
  digitalWrite(MOTX_DIR, HIGH);
  TCCR1B = (TCCR1B & 0b11111000) | 0x01; //31372.55Hz https://playground.arduino.cc/Main/TimerPWMCheatsheet
  TCCR2B = (TCCR2B & 0b11111000) | 0x01; //31372.55Hz https://playground.arduino.cc/Main/TimerPWMCheatsheet
  Serial.begin(9600);
}

// the loop routine runs over and over again forever:
void loop() {
  analogWrite(MOTX_SPD, brightness);
  Serial.print(brightness);
  delay(10);
  int freqV = analogRead(MOTX_FREQ);
  Serial.print(" ");
  Serial.print(freqV);
  delay(10);
  freqV = analogRead(MOTX_FREQ);
  Serial.print(" ");
  Serial.print(freqV);
  delay(10);
  freqV = analogRead(MOTX_FREQ);
  Serial.print(" ");
  Serial.println(freqV);
  
  brightness = brightness + fadeAmount;
  if (brightness <= 0 || brightness >= 255) {
    fadeAmount = -fadeAmount;
  }
  delay(50);
}
