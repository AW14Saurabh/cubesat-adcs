/*******************************************************************************
 * This library is an interface to the L3GD20 Gyroscope connected to the board
 * on Analog pins Pin 18 A4 SDA and Pin 19 A5 SCL on the I2C bus using the
 * SoftI2C library.
 ******************************************************************************/
#ifndef __ATTITUDE_DETERMINATION_H__
#define __ATTITUDE_DETERMINATION_H__

#define ID 20
#define MIN_SAMPLE_TIME 30

#include <Adafruit_L3GD20_U.h>
#include "Data.h"

class Attitude_Determination
{
private:
    Adafruit_L3GD20_Unified _gyro;
    angVelData_t _bias;
    bool _anglesComputed;
    float inverseSqrt(float);

public:
    Attitude_Determination();

    void getAngles(angRPYData_t*, attdData_t*);
    void updateHeading(angVelData_t*, attdData_t*, int32_t);
};

#endif