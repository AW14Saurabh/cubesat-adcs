/*******************************************************************************
 * This library is an interface to the NRF24L01 Radio Module connected on the
 * SPI bus on pins 10(SS/CS), 11(MOSI), 12(MISO), 13(SCK) using the RF24 Library
 ******************************************************************************/
#ifndef __RADIO_COMMUNICATION_H__
#define __RADIO_COMMUNICATION_H__

#include <SPI.h>
#include <RF24.h>

#define NRF_CE 9
#define NRF_CS 10

#define DISABLED 0
#define ENABLED 1

#define DETUMBLE 0
#define POINT 1

#define UPDATE_TIME 50

/*==============================================================================
    DATA TYPE FOR MESSAGE PACKET
    --------------------------------------------------------------------------*/
    typedef struct messageData_s
    {
        int enableState;
        int mode;
        float targetAngle;
    } messageData_t;
/*============================================================================*/

class Radio_Communication
{
private:
    RF24          _radio;
    uint32_t      _messageIn;
    messageData_t _message;
    const uint8_t _rxAddr[6];

public:
    Radio_Communication();
    messageData_t getMessage();
};
#endif