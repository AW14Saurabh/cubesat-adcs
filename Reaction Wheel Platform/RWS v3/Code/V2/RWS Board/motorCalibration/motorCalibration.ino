

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

unsigned long startmillis;
unsigned long prevmillis;

void setup() {
  pinMode(MOT1_SPD, OUTPUT);
  pinMode(MOT1_DIR, OUTPUT);
  pinMode(MOT1_FREQ, INPUT);
  pinMode(MOT2_SPD, OUTPUT);
  pinMode(MOT2_DIR, OUTPUT);
  pinMode(MOT2_FREQ, INPUT);
  pinMode(MOT3_SPD, OUTPUT);
  pinMode(MOT3_DIR, OUTPUT);
  pinMode(MOT3_FREQ, INPUT);
  pinMode(MOT4_SPD, OUTPUT);
  pinMode(MOT4_DIR, OUTPUT);
  pinMode(MOT4_FREQ, INPUT);
  digitalWrite(MOT1_DIR, HIGH);
  digitalWrite(MOT2_DIR, HIGH);
  digitalWrite(MOT3_DIR, HIGH);
  digitalWrite(MOT4_DIR, HIGH);
  analogWrite(MOT1_SPD, 150);
  analogWrite(MOT2_SPD, 150);
  analogWrite(MOT3_SPD, 150);
  analogWrite(MOT4_SPD, 150);
  TCCR1B = (TCCR1B & 0b11111000) | 0x01; //31372.55Hz https://playground.arduino.cc/Main/TimerPWMCheatsheet
  TCCR2B = (TCCR2B & 0b11111000) | 0x01; //31372.55Hz https://playground.arduino.cc/Main/TimerPWMCheatsheet
  Serial.begin(9600);
  delay(4000);
  startmillis = millis();
  prevmillis = millis();
}

int mot1FreqV;
int mot2FreqV;
int mot3FreqV;
int mot4FreqV;

// the loop routine runs over and over again forever:
void loop() {
  unsigned long currentmillis = millis();
  int spd = 150-(int)((float)(currentmillis-startmillis)*0.315);
  analogWrite(MOT1_SPD, spd);
  analogWrite(MOT2_SPD, spd);
  analogWrite(MOT3_SPD, spd);
  analogWrite(MOT4_SPD, spd);
  mot1FreqV = analogRead(MOT1_FREQ);
  mot2FreqV = analogRead(MOT2_FREQ);
  mot3FreqV = analogRead(MOT3_FREQ);
  mot4FreqV = analogRead(MOT4_FREQ);
  Serial.print(spd);
  Serial.print(" ");
  Serial.print(mot1FreqV);
  Serial.print(" ");
  Serial.print(mot2FreqV);
  Serial.print(" ");
  Serial.print(mot3FreqV);
  Serial.print(" ");
  Serial.print(mot4FreqV);
  Serial.println();
  if (spd <= 0){
    spd = 255;
    analogWrite(MOT1_SPD, spd);
    analogWrite(MOT2_SPD, spd);
    analogWrite(MOT3_SPD, spd);
    analogWrite(MOT4_SPD, spd);
    delay(100000);
  }
  delay(10);
  
  /*//255-0, 0-255
  analogWrite(MOT1_SPD, 0);
  analogWrite(MOT2_SPD, 0);
  analogWrite(MOT3_SPD, 0);
  analogWrite(MOT4_SPD, 0);
  int mot1FreqV;
  int mot2FreqV;
  int mot3FreqV;
  int mot4FreqV;
  unsigned long prevmillis = millis();
  while (millis()-prevmillis < 4000){
    mot1FreqV = analogRead(MOT1_FREQ);
    mot2FreqV = analogRead(MOT2_FREQ);
    mot3FreqV = analogRead(MOT3_FREQ);
    mot4FreqV = analogRead(MOT4_FREQ);
    Serial.print(mot1FreqV);
    Serial.print(" ");
    Serial.print(mot2FreqV);
    Serial.print(" ");
    Serial.print(mot3FreqV);
    Serial.print(" ");
    Serial.print(mot4FreqV);
    Serial.println();
    delay(10);
  }
  Serial.println("TURNING OFF MOTORS");
  analogWrite(MOT1_SPD, 255);
  analogWrite(MOT2_SPD, 255);
  analogWrite(MOT3_SPD, 255);
  analogWrite(MOT4_SPD, 255);
  prevmillis = millis();
  while (millis()-prevmillis < 4000){
    mot1FreqV = analogRead(MOT1_FREQ);
    mot2FreqV = analogRead(MOT2_FREQ);
    mot3FreqV = analogRead(MOT3_FREQ);
    mot4FreqV = analogRead(MOT4_FREQ);
    Serial.print(mot1FreqV);
    Serial.print(" ");
    Serial.print(mot2FreqV);
    Serial.print(" ");
    Serial.print(mot3FreqV);
    Serial.print(" ");
    Serial.print(mot4FreqV);
    Serial.println();
    delay(10);
  }
  delay(10000);
  */
}
