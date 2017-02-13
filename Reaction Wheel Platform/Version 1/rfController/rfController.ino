#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(9, 10);

const byte rxAddr[6] = "00001";

void setup()
{
  Serial.begin(9600);

  
  radio.begin();
  radio.setRetries(15, 15);
  radio.openWritingPipe(rxAddr);
  radio.stopListening();
}

float switchA = 0;
float pot = 0;
void loop()
{
  switchA = digitalRead(A0);
  pot = analogRead(A1)/1024.0*(2*PI);
  float tx = switchA*(2*PI) + pot;
    Serial.println(tx);
  radio.write(&tx, sizeof(tx));
  delay(10);
}
