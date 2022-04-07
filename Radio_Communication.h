/*******************************************************************************
 * This library is an interface to the NRF24L01 Radio Module connected on the
 * SPI bus on pins 10(SS/CS), 11(MOSI), 12(MISO), 13(SCK) using the RF24 Library
 ******************************************************************************/
#ifndef __RADIO_COMMUNICATION_H__
#define __RADIO_COMMUNICATION_H__

#include <SPI.h>
#include <RF24.h>
#include "Data.h"

#define NRF_CE 7
#define NRF_CS 8

class Radio_Communication
{
private:
    RF24 _radio;
    messageData_t _message;
    const uint8_t _rxAddr[2][6];

public:
    Radio_Communication();
    messageData_t getMessage();
    void sendMessage(angRPYData_t*);
};
#endif