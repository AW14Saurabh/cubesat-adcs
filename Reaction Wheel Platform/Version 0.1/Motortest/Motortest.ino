#define FWD 1
#define REV -1

void setup() {
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
}
int i = 0;
int dir = FWD;

// the loop function runs over and over again forever
void loop() {


  if (i > 255){
    dir = REV;
    i--;   
  }
  if (i < -255){
    dir = FWD;
    i++;
  }

  if (dir == FWD){
    i++;
  } else {
    i--;
  }

  if (i>=0){
    analogWrite(5, i);
    digitalWrite(6, LOW);
  } else {
    digitalWrite(5, LOW);
    analogWrite(6, -i);
  }

  delay(1);


  /*
  digitalWrite(5, HIGH);
  digitalWrite(6, LOW);
  delay(100);
  digitalWrite(5, LOW);
  digitalWrite(6, LOW);
  delay(100);
  digitalWrite(5, LOW);
  digitalWrite(6, HIGH);
  delay(100);
  digitalWrite(5, LOW);
  digitalWrite(6, LOW);
  delay(100);*/
}
