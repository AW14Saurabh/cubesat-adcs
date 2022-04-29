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
Attitude_Determination::Attitude_Determination(angVelData_t *bias) : _gyro(Adafruit_L3GD20_Unified(ID)),
                                                   _bias(bias),
                                                   _anglesComputed(false)
{
    /* Initialize Gyro */
    _gyro.enableAutoRange(true);
    _gyro.begin();

    /* Calibrate Gyro */
    for (int i = 0; i < 1000; i++)
    {
        sensors_event_t event;
        _gyro.getEvent(&event);
        _bias->x += event.gyro.x;
        _bias->y += event.gyro.y;
        _bias->z += event.gyro.z;
        delay(10);
    }
    _bias->x /= 1000;
    _bias->y /= 1000;
    _bias->z /= 1000;
    // Serial.println("Bias:\t" + String(_bias.x) + "\t" + String(_bias.y) + "\t" + String(_bias.z));
}

/*******************************************************************************
 PUBLIC FUNCTIONS
 ******************************************************************************/

/******************************************************************************/
/*!
    @brief  Calculate the roll, pitch and yaw in degrees
*/
/******************************************************************************/
void Attitude_Determination::getAngles(angRPYData_t *ang, attdData_t *attd)
{
    if (!_anglesComputed)
    {
        ang->x = atan2f(attd->a * attd->b + attd->c * attd->d, 0.5f - attd->b * attd->b - attd->c * attd->c);
        ang->y = asinf(-2.0f * (attd->b * attd->d - attd->a * attd->c));
        ang->z = atan2f(attd->b * attd->c + attd->a * attd->d, 0.5f - attd->c * attd->c - attd->d * attd->d);
        _anglesComputed = true;
    }

    ang->x *= 57.29578f;
    ang->y *= 57.29578f;
    ang->z *= 57.29578f;
}

/******************************************************************************/
/*!
    @brief  Update the Heading from gyro reading
*/
/******************************************************************************/
void Attitude_Determination::updateHeading(angVelData_t *angVel, attdData_t *attd, int32_t dt)
{
    float normal;
    angVelData_t tmpAngVel;
    attdData_t tmpAtt = {attd->a, attd->b, attd->c, 0.0};
    float dtSec = (float)dt / 1000;
    sensors_event_t eventG;

    _gyro.getEvent(&eventG);
    angVel->x = eventG.gyro.x;
    angVel->y = eventG.gyro.y;
    angVel->z = eventG.gyro.z;

    /* Integrate rate of change of quaternions */
    tmpAngVel.x = angVel->x * (0.5f * dtSec);
    tmpAngVel.y = angVel->y * (0.5f * dtSec);
    tmpAngVel.z = angVel->z * (0.5f * dtSec);

    attd->a += (-tmpAtt.b * tmpAngVel.x - tmpAtt.c * tmpAngVel.y - attd->d * tmpAngVel.z);
    attd->b += (tmpAtt.a * tmpAngVel.x + tmpAtt.c * tmpAngVel.z - attd->d * tmpAngVel.y);
    attd->c += (tmpAtt.a * tmpAngVel.y - tmpAtt.b * tmpAngVel.z - attd->d * tmpAngVel.x);
    attd->d += (tmpAtt.a * tmpAngVel.z + tmpAtt.b * tmpAngVel.y - tmpAtt.c * tmpAngVel.x);

    /* Normalize the quaternions */
    normal = inverseSqrt(attd->a * attd->a + attd->b * attd->b + attd->c * attd->c + attd->d * attd->d);
    attd->a *= normal;
    attd->b *= normal;
    attd->c *= normal;
    attd->d *= normal;
    _anglesComputed = false;
}