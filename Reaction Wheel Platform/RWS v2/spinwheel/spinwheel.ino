//Just spins wheel at constant speed; for o-week stuff

#define VR 5
#define ZF 7


void setup(void){
  pinMode(9, OUTPUT);
  pinMode(7, OUTPUT);
}


void loop(void){
  digitalWrite(ZF, HIGH);
  analogWrite(VR, 50);
  delay(5000);
}



