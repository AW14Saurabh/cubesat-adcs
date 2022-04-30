/*******************************************************************************
 * This library is an interface to the NRF24L01 Radio Module connected on the
 * SPI bus on pins 10(SS/CS), 11(MOSI), 12(MISO), 13(SCK) using the RF24 Library
 ******************************************************************************/
#include "Radio_Communication.h"

/*******************************************************************************
 CONSTRUCTOR
 ******************************************************************************/

/******************************************************************************/
/*!
    @brief  Instantiates a new Radio_Communication class
*/
/******************************************************************************/
Radio_Communication::Radio_Communication() : _radio(RF24(NRF_CE, NRF_CS))
{
    pinMode(NRF_CE, OUTPUT);
    pinMode(NRF_CS, OUTPUT);
    _radio.begin();
    _radio.setPALevel(RF24_PA_LOW);  // RF24_PA_MAX is default.
    _radio.setRetries(15, 15);
    _radio.openWritingPipe(_rxAddr[1]);
    _radio.openReadingPipe(1, _rxAddr[0]);
    // _radio.startListening();
}

/*******************************************************************************
 PUBLIC FUNCTIONS
 ******************************************************************************/

/******************************************************************************/
/*!
    @brief  Get message from Control Box
*/
/******************************************************************************/
void Radio_Communication::getMessage(messageData_t *message)
{
    delay(5);
    _radio.startListening();
    while(!_radio.available())
    {
        // Serial.println("Waiting for radio");
    };
    _radio.read(message, sizeof(messageData_t));
}

/******************************************************************************/
/*!
    @brief  Send angles to Control Box
*/
/******************************************************************************/
void Radio_Communication::sendMessage(angRPYData_t* satAngles)
{
    delay(5);
    _radio.stopListening();
    _radio.write(satAngles, sizeof(angRPYData_t));
}