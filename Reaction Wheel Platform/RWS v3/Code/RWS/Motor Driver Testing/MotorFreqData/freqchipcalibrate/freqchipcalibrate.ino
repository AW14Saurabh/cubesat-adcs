

#define FREQ_OUT 2
#define FREQ_IN A3
#define PRINT_TIME 1000000 //ms (based off processing time between subsequent loops) //100ms w/ serial print



void setup() {
  pinMode(FREQ_OUT, OUTPUT);
  pinMode(FREQ_IN, INPUT);
  Serial.begin(9600);
}



unsigned long prevMicrosPrint = 0;
unsigned long prevMicrosFreq = 0;
bool currOut = false;
unsigned long actualDtFreq = 0;



// the loop routine runs over and over again forever:
void loop() {
  unsigned long currentMicros = micros();
  
  
  if (currentMicros - prevMicrosFreq >= (currentMicros/1000000)*50+500) {
    actualDtFreq = currentMicros - prevMicrosFreq;
    prevMicrosFreq = currentMicros;
    if (currOut){
      digitalWrite(FREQ_OUT, HIGH);
      currOut = false;
    } else {
      digitalWrite(FREQ_OUT, LOW);
      currOut = true;
    }
  }
  
  if (currentMicros - prevMicrosPrint >= PRINT_TIME) {
    prevMicrosPrint = currentMicros;
    int freqV = analogRead(FREQ_IN);
    Serial.print(actualDtFreq);
    Serial.print(" ");
    Serial.print(freqV);
    freqV = analogRead(FREQ_IN);
    Serial.print(" ");
    Serial.print(freqV);
    Serial.println();
  }

  
}
