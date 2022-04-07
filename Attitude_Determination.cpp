/*******************************************************************************
 * This library is an interface to the L3GD20 Gyroscope connected to the board
 * on Analog pins Pin 18 A4 SDA and Pin 19 A5 SCL on the I2C bus using the
 * SoftI2C library.
 ******************************************************************************/
#include "Attitude_Determination.h"

/*******************************************************************************
 PRIVATE FUNCTIONS
 ******************************************************************************/

/******************************************************************************/
/*!
    @brief  Compute angle in radians from the quaternions
*/
/******************************************************************************/
void Attitude_Determination::computeAngles()
{
    _anglesRadian.x = atan2f(_attitude.a * _attitude.b + _attitude.c * _attitude.d,
                             0.5f - _attitude.b * _attitude.b - _attitude.c * _attitude.c);

    _anglesRadian.y = asinf(-2.0f * (_attitude.b * _attitude.d - _attitude.a * _attitude.c));

    _anglesRadian.z = atan2f(_attitude.b * _attitude.c + _attitude.a * _attitude.d,
                             0.5f - _attitude.c * _attitude.c - _attitude.d * _attitude.d);

    _anglesComputed = true;
}

/******************************************************************************/
/*!
    @brief  See: https://en.wikipedia.org/wiki/Fast_inverse_square_root
*/
/******************************************************************************/
float Attitude_Determination::inverseSqrt (float x)
{
    long i;
    float x2, y;
    const float threehalfs = 1.5f;

    x2 = x * 0.5f;
    y = x;
    i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *)&i;
    y = y * (threehalfs - x2 * y * y);
    y = y * (threehalfs - x2 * y * y);
    return y;
}

/*******************************************************************************
 CONSTRUCTOR
 ******************************************************************************/

/******************************************************************************/
/*!
    @brief  Instantiates a new Attitude_Determination class
*/
/******************************************************************************/
Attitude_Determination::Attitude_Determination() : _gyro(Adafruit_L3GD20_Unified(ID)),
                                                   _angVel{0.0, 0.0, 0.0},
                                                   _bias{0.0, 0.0, 0.0},
                                                   _anglesRadian{0.0, 0.0, 0.0},
                                                   _anglesDegree{0.0, 0.0, 0.0},
                                                   _attitude{1.0, 0.0, 0.0, 0.0},
                                                   _anglesComputed(false)
{
    /* Initialize Gyro */
    _gyro.enableAutoRange(true);
    _gyro.begin();
    delay(1000);

    /* Calibrate Gyro */
    for (int i = 0; i < 1000; i++)
    {
        sensors_event_t event;
        _gyro.getEvent(&event);
        _bias.x += event.gyro.x;
        _bias.y += event.gyro.y;
        _bias.z += event.gyro.z;
        delay(10);
    }
    _bias.x /= 1000;
    _bias.y /= 1000;
    _bias.z /= 1000;
}

/*******************************************************************************
 PUBLIC FUNCTIONS
 ******************************************************************************/

/******************************************************************************/
/*!
    @brief  Calculate the roll, pitch and yaw in degrees
*/
/******************************************************************************/
angRPYData_t Attitude_Determination::getAngles()
{
    if (!_anglesComputed)
        computeAngles();
    _anglesDegree.x = _anglesRadian.x * 57.29578f;
    _anglesDegree.y = _anglesRadian.y * 57.29578f;
    _anglesDegree.z = _anglesRadian.z * 57.29578f + 180.0f;
    return _anglesDegree;
}

/******************************************************************************/
/*!
    @brief  Update the Heading from gyro reading
*/
/******************************************************************************/
dataPacket_t Attitude_Determination::updateHeading(int32_t dt)
{
    float normal;
    angVelData_t tmpAngVel;
    attdData_t tmpAtt = {_attitude.a, _attitude.b, _attitude.c, 0.0};
    float dtSec = (float)dt / 1000;
    dataPacket_t heading;
    sensors_event_t eventG;

    _gyro.getEvent(&eventG);
    _angVel.x = eventG.gyro.x - _bias.x;
    _angVel.y = eventG.gyro.y - _bias.y;
    _angVel.z = eventG.gyro.z - _bias.z;

    heading.angVel = _angVel;

    /* Integrate rate of change of quaternions */
    tmpAngVel.x = _angVel.x * (0.5f * dtSec);
    tmpAngVel.y = _angVel.y * (0.5f * dtSec);
    tmpAngVel.z = _angVel.z * (0.5f * dtSec);

    _attitude.a += (-tmpAtt.b * tmpAngVel.x - tmpAtt.c * tmpAngVel.y - _attitude.d * tmpAngVel.z);
    _attitude.b += (tmpAtt.a * tmpAngVel.x + tmpAtt.c * tmpAngVel.z - _attitude.d * tmpAngVel.y);
    _attitude.c += (tmpAtt.a * tmpAngVel.y - tmpAtt.b * tmpAngVel.z - _attitude.d * tmpAngVel.x);
    _attitude.d += (tmpAtt.a * tmpAngVel.z + tmpAtt.b * tmpAngVel.y - tmpAtt.c * tmpAngVel.x);

    /* Normalize the quaternions */
    normal = inverseSqrt(_attitude.a * _attitude.a +
                         _attitude.b * _attitude.b +
                         _attitude.c * _attitude.c +
                         _attitude.d * _attitude.d);
    _attitude.a *= normal;
    _attitude.b *= normal;
    _attitude.c *= normal;
    _attitude.d *= normal;
    heading.attitude = _attitude;
    _anglesComputed = false;

    return heading;
}