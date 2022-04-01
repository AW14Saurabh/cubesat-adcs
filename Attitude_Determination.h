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

/*==============================================================================
    DATA TYPE FOR AXES
    --------------------------------------------------------------------------*/
    typedef struct axesData_s
    {
        float x;
        float y;
        float z;
    } axesData_t;
/*============================================================================*/

/*==============================================================================
    DATA TYPE FOR QUATERNION
    --------------------------------------------------------------------------*/
    typedef struct quaternionData_s
    {
        float a;
        float b;
        float c;
        float d;
    } quaternionData_t;
/*============================================================================*/

/*==============================================================================
    DATA TYPE FOR GYRO EVENT
    --------------------------------------------------------------------------*/
    typedef struct dataPacket_s
    {
        axesData_t spin;
        quaternionData_t attitude;
    } dataPacket_t;
/*============================================================================*/

class Attitude_Determination
{
    public:
    Attitude_Determination();

    dataPacket_t updateHeading ( int32_t dt );
    axesData_t getAngles       ();

    private:
    Adafruit_L3GD20_Unified _gyro;
    axesData_t              _spin;
    axesData_t              _bias;
    axesData_t              _anglesRadian;
    axesData_t              _anglesDegree;
    quaternionData_t        _attitude;
    dataPacket_t            _message;
    bool                    _anglesComputed;
    void                    computeAngles   ();
    static float            inverseSqrt     ( float x )
    {
        long i;
        float x2, y;
        const float threehalfs = 1.5f;

        x2 = x * 0.5f;
        y  = x;
        i  = * ( long * ) &y;
        i  = 0x5f3759df - (i >> 1);
        y  = * ( float * ) &i;
        y  = y * ( threehalfs - x2 * y * y);
        y  = y * ( threehalfs - x2 * y * y);
        return y;
    }
};

#endif