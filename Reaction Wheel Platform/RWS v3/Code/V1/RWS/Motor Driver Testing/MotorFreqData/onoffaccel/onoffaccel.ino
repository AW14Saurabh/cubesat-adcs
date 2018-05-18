

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
  analogWrite(MOT1_SPD, 255);
}

// the loop routine runs over and over again forever:
void loop() {
  int freqV;
  int i;
  delay(1000);
  Serial.println("255-0");
  analogWrite(MOT1_SPD, 0);
  for (i = 0; i < 300; i++){
    freqV = analogRead(A3);
    Serial.println(freqV);
    delay(10);
  }
  delay(1000);

  Serial.println("0-(-0)");
  digitalWrite(MOT1_DIR, LOW);
  for (i = 0; i < 300; i++){
    freqV = analogRead(A3);
    Serial.println(freqV);
    delay(10);
  }
  delay(1000);

  
  Serial.println("(-0)-0");
  digitalWrite(MOT1_DIR, HIGH);
  for (i = 0; i < 300; i++){
    freqV = analogRead(A3);
    Serial.println(freqV);
    delay(10);
  }
  delay(1000);

  Serial.println("0-255");
  analogWrite(MOT1_SPD, 255);
  for (i = 0; i < 300; i++){
    freqV = analogRead(A3);
    Serial.println(freqV);
    delay(10);
  }
}
