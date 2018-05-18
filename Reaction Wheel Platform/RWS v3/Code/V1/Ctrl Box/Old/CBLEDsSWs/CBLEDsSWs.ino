/*
  Blink
  Turns on an LED on for one second, then off for one second, repeatedly.

  Most Arduinos have an on-board LED you can control. On the UNO, MEGA and ZERO 
  it is attached to digital pin 13, on MKR1000 on pin 6. 6 is set to
  the correct LED pin independent of which board is used.
  If you want to know what pin the on-board LED is connected to on your Arduino model, check
  the Technical Specs of your board  at https://www.arduino.cc/en/Main/Products
  
  This example code is in the public domain.

  modified 8 May 2014
  by Scott Fitzgerald
  
  modified 2 Sep 2016
  by Arturo Guadalupi
  
  modified 8 Sep 2016
  by Colby Newman
*/
#define LED_R 5
#define LED_G 6
#define LED_B 9
#define SW_L  8
#define SW_R  4
#define PB    7

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin 6 as an output.
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  pinMode(SW_L, INPUT);
  pinMode(SW_R, INPUT);
  pinMode(PB, INPUT);
}

// the loop function runs over and over again forever
void loop() {
  if (digitalRead(SW_L) == HIGH){
    digitalWrite(LED_R, HIGH);   // turn the LED on (HIGH is the voltage level)
  } else {
    digitalWrite(LED_R, LOW);   // turn the LED on (HIGH is the voltage level)  
  }
  if (digitalRead(SW_R) == HIGH){
    digitalWrite(LED_G, HIGH);   // turn the LED on (HIGH is the voltage level)
  } else {
    digitalWrite(LED_G, LOW);   // turn the LED on (HIGH is the voltage level)  
  }
  if (digitalRead(PB) == HIGH){
    digitalWrite(LED_B, HIGH);   // turn the LED on (HIGH is the voltage level)
  } else {
    digitalWrite(LED_B, LOW);   // turn the LED on (HIGH is the voltage level)  
  }

}
