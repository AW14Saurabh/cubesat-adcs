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
Radio_Communication::Radio_Communication() : _radio(RF24(NRF_CE, NRF_CS)),
                                             _message{0, 0, 0.0},
                                             _rxAddr{"00001", "00002"}
{
    pinMode(NRF_CE, OUTPUT);
    pinMode(NRF_CS, OUTPUT);
    _radio.begin();
    _radio.openWritingPipe(_rxAddr[1]);
    _radio.openReadingPipe(0, _rxAddr[0]);
    _radio.startListening();
}

/*******************************************************************************
 PUBLIC FUNCTIONS
 ******************************************************************************/

/******************************************************************************/
/*!
    @brief  Get message from Control Box
*/
/******************************************************************************/
messageData_t Radio_Communication::getMessage()
{
    if (_radio.available())
    {
        delay(5);
        _radio.startListening();
        if (_radio.available())
            _radio.read(&_message, sizeof(messageData_t));
    }
    return _message;
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