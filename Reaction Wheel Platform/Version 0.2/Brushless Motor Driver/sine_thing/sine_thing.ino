
int A = 5;
int B = 6;
int C = 9;
int pause = 1;
int Aval = 0;
int Bval = 127;
int Cval = 255;
int theta = 0; //in degrees
int alpha = 120;
int beta = 240;
int incr = 10;

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(A, OUTPUT);
  pinMode(B, OUTPUT);
  pinMode(C, OUTPUT);
  Serial.begin(9600);

}

// the loop function runs over and over again forever
void loop() {

  Aval = round(255*(    sin(theta      *PI/180)   +1)/2);
  Bval = round(255*(    sin((alpha)*PI/180)   +1)/2);
  Cval = round(255*(    sin((beta)*PI/180)   +1)/2);
  Serial.println(Aval);
  Serial.println(Bval);
  Serial.println(Cval);
  Serial.println(99999999);

  theta = theta + incr;
  beta = beta +incr;
  alpha = alpha +incr;

  if(theta > 360){
    theta = 0;
  }

  if(alpha >360){
    alpha = 0;
  }
  if(beta >360){
    beta = 0;
  }
  
  
  analogWrite(A, Aval);
  analogWrite(B, Bval);
  analogWrite(C, Cval);
  
  delay(pause);

}

