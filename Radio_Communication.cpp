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
                                             _rxAddr("00065")
{
    pinMode(NRF_CE, OUTPUT);
    pinMode(NRF_CS, OUTPUT);
    _radio.begin();
    _radio.openReadingPipe(0, _rxAddr);
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
        _radio.read(&_messageIn, sizeof(_messageIn));
        _message.enableState = _messageIn >> 15;
        _messageIn &= (uint32_t)B01111111 * 256 + B11111111;
        _message.mode = _messageIn >> 14;
        _messageIn &= (uint32_t)B10111111 * 256 + B11111111;
        _message.targetAngle = constrain(_messageIn * 2 * PI / 16383.0, 0, 2 * PI);
    }
    return _message;
}
