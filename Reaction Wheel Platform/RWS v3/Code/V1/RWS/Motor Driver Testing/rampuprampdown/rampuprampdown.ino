

#define MOT1_SPD 3
#define MOT1_DIR 4


int brightness = 0;    // how bright the spd is
int fadeAmount = 1;    // how many points to fade the spd by


// the setup routine runs once when you press reset:
void setup() {
  // declare pin 9 to be an output:
  pinMode(MOT1_SPD, OUTPUT);
  pinMode(MOT1_DIR, OUTPUT);
  digitalWrite(MOT1_DIR, HIGH);
  TCCR2B = (TCCR2B & 0b11111000) | 0x01; //31372.55Hz https://playground.arduino.cc/Main/TimerPWMCheatsheet
  Serial.begin(9600);
}

// the loop routine runs over and over again forever:
void loop() {
  analogWrite(MOT1_SPD, brightness);
  Serial.print(brightness);
  delay(10);
  int freqV = analogRead(A3);
  Serial.print(" ");
  Serial.print(freqV);
  delay(10);
  freqV = analogRead(A3);
  Serial.print(" ");
  Serial.print(freqV);
  delay(10);
  freqV = analogRead(A3);
  Serial.print(" ");
  Serial.println(freqV);
  
  brightness = brightness + fadeAmount;
  if (brightness <= 0 || brightness >= 255) {
    fadeAmount = -fadeAmount;
  }
  delay(50);
}
