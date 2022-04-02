/*******************************************************************************
 * This library is an interface to the L3GD20 Gyroscope connected to the board
 * on Analog pins Pin 18 A4 SDA and Pin 19 A5 SCL on the I2C bus using the
 * SoftI2C library.
 ******************************************************************************/
#ifndef __ATTITUDE_DETERMINATION_H__
#define __ATTITUDE_DETERMINATION_H__

#define ID 20
#define MIN_SAMPLE_TIME 30

#include <Adafruit_L3GD20_U_SOFTI2C.h>
#include "Data.h"

class Attitude_Determination
{
private:
    Adafruit_L3GD20_Unified _gyro;
    angVelData_t _angVel;
    angVelData_t _bias;
    angRPYData_t _anglesRadian;
    angRPYData_t _anglesDegree;
    attdData_t _attitude;
    bool _anglesComputed;
    void computeAngles();
    float inverseSqrt(float);

public:
    Attitude_Determination();

    dataPacket_t updateHeading(int32_t);
    angRPYData_t getAngles();
};

#endif