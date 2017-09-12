

// constants won't change. They're used here to
// set pin numbers:
const int btnPin = 9;     // the number of the pushbutton pin
const int redPin =  5;
const int grnPin =  6;

// variables will change:
int btnState = 0;         // variable for reading the pushbutton status

void setup() {
  // initialize the LED pin as an output:
  pinMode(redPin, OUTPUT);
  pinMode(grnPin, OUTPUT);
  // initialize the pushbutton pin as an input:
  pinMode(btnPin, INPUT);
  Serial.begin(9600);

}

void loop() {
  // read the state of the pushbutton value:
  btnState = digitalRead(btnPin);
  Serial.println(btnState);

  if (btnState == HIGH) {
    digitalWrite(redPin, HIGH);
    digitalWrite(grnPin, LOW);
  } else {
    digitalWrite(redPin, LOW);
    digitalWrite(grnPin, HIGH);
  }
}
