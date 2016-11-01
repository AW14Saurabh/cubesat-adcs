
int A = 5;
int B = 6;
int C = 9;
int pause = 5;

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(A, OUTPUT);
  pinMode(B, OUTPUT);
  pinMode(C, OUTPUT);
}

// the loop function runs over and over again forever
void loop() {
  //1
  analogWrite(A, 0);
  analogWrite(B, 127);
  analogWrite(C, 255);
  delay(pause);

  //2
  analogWrite(A, 127);
  analogWrite(B, 0);
  analogWrite(C, 255);
  delay(pause);

  //3
  analogWrite(A, 255);
  analogWrite(B, 0);
  analogWrite(C, 127);
  delay(pause);

  //4
  analogWrite(A, 255);
  analogWrite(B, 127);
  analogWrite(C, 0);
  delay(pause);

  //5
  analogWrite(A, 127);
  analogWrite(B, 255);
  analogWrite(C, 0);
  delay(pause);

  //6
  analogWrite(A, 0);
  analogWrite(B, 255);
  analogWrite(C, 127);
  delay(pause);

//  pause = analogRead(A2);

}
