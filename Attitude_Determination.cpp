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
Attitude_Determination::Attitude_Determination() : _gyro(Adafruit_L3GD20_Unified(ID)),
                                                   _biasVel({0.0, 0.0, 0.0}),
                                                   _biasAng({0.0,0.0,0.0}),
                                                   _anglesComputed(false)
{
    /* Initialize Gyro */
    _gyro.enableAutoRange(true);
    _gyro.begin();

    /* Calibrate Gyro */
    for (int i = 0; i < 500; i++)
    {
        sensors_event_t event;
        _gyro.getEvent(&event);
        _biasVel.x += event.gyro.x;
        _biasVel.y += event.gyro.y;
        _biasVel.z += event.gyro.z;
        delay(10);
        _biasAng.x += _biasVel.x * 0.01;
        _biasAng.y += _biasVel.y * 0.01;
        _biasAng.z += _biasVel.z * 0.01;
    }
    _biasVel.x /= 500;
    _biasVel.y /= 500;
    _biasVel.z /= 500;
    _biasAng.x /= 500;
    _biasAng.y /= 500;
    _biasAng.z /= 500;
    // Serial.println("Bias:\t" + String(_biasVel.x) + "\t" + String(_biasVel.y) + "\t" + String(_biasVel.z));
}

/*******************************************************************************
 PUBLIC FUNCTIONS
 ******************************************************************************/

/******************************************************************************/
/*!
    @brief  Calculate the roll, pitch and yaw in degrees
*/
/******************************************************************************/
void Attitude_Determination::getAngles(angRPYData_t *ang, angVelData_t *w, int32_t dt)
{
    float dtSec = (float)dt / 1000.0;
    if (!_anglesComputed)
    {
        // ang->x = atan2(q->a*q->b + q->c*q->d, 0.5f - q->b*q->b - q->c*q->c);
        // ang->y = asin(-2.0f*(q->b*q->d - q->a*q->c));
        // ang->z = atan2(q->b*q->c + q->a*q->d, 0.5f - q->c*q->c - q->d*q->d);
        ang->x += (w->x * dtSec) - _biasAng.x;
        ang->y += (w->y * dtSec) - _biasAng.y;
        ang->z += (w->z * dtSec) - _biasAng.z;
        _anglesComputed = true;
    }
    Serial.println("AnglesRad:\t" + String(ang->x, 6) + "\t" + String(ang->y, 6) + "\t" + String(ang->z, 6));
    // ang->x *= 57.29578f;
    // ang->y *= 57.29578f;
    // ang->z *= 57.29578f;
}

/******************************************************************************/
/*!
    @brief  Update the Heading from gyro reading
*/
/******************************************************************************/
void Attitude_Determination::updateHeading(angVelData_t *w, attdData_t *q, int32_t dt)
{
    float normal;
    angVelData_t tw {0.0, 0.0, 0.0};
    attdData_t tq {q->a, q->b, q->c, q->d};
    float dtSec = (float)dt / 1000;
    // Serial.println("Delta: " + String(dtSec));
    sensors_event_t eventG;

    _gyro.getEvent(&eventG);
    w->x = eventG.gyro.x - _biasVel.x;
    w->y = eventG.gyro.y - _biasVel.y;
    w->z = eventG.gyro.z - _biasVel.z;

    tw.x = w->x;
    tw.y = w->y;
    tw.z = w->z;

    /* Integrate rate of change of quaternions */
    /* Pre-multiply common factors */
    w->x *= (0.5f * dtSec);
    w->y *= (0.5f * dtSec);
    w->z *= (0.5f * dtSec);

    q->a += (-tq.b*tw.x - tq.c*tw.y - tq.d*tw.z);
    q->b += ( tq.a*tw.x + tq.c*tw.z - tq.d*tw.y);
    q->c += ( tq.a*tw.y - tq.b*tw.z + tq.d*tw.x);
    q->d += ( tq.a*tw.z + tq.b*tw.y - tq.c*tw.x);

    /* Normalize the quaternions */
    normal = inverseSqrt(q->a*q->a + q->b*q->b + q->c*q->c + q->d*q->d);
    q->a *= normal;
    q->b *= normal;
    q->c *= normal;
    q->d *= normal;
    _anglesComputed = false;
}