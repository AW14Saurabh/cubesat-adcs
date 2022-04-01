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

/*******************************************************************************
 CONSTRUCTOR
 ******************************************************************************/

/******************************************************************************/
/*!
    @brief  Instantiates a new Attitude_Determination class
*/
/******************************************************************************/
Attitude_Determination::Attitude_Determination()
{
    _gyro = Adafruit_L3GD20_Unified(20);
    _spin = {0.0, 0.0, 0.0};
    _bias = {0.0, 0.0, 0.0};
    _anglesRadian = {0.0, 0.0, 0.0};
    _anglesDegree = {0.0, 0.0, 0.0};

    /* Reset Heading */
    _attitude = {1.0, 0.0, 0.0, 0.0};
    _anglesComputed = false;

    /* Initialize Gyro */
    _gyro.enableAutoRange(true);
    _gyro.begin();
    delay(1000);

    /* Calibrate Gyro */
    for (int i = 0; i < 100; i++)
    {
        sensors_event_t event;
        _gyro.getEvent(&event);
        _bias.x += event.gyro.x;
        _bias.y += event.gyro.y;
        _bias.z += event.gyro.z;
        delay(10);
    }
    _bias.x /= 100;
    _bias.y /= 100;
    _bias.z /= 100;
}

/*******************************************************************************
 PUBLIC FUNCTIONS
 ******************************************************************************/

/******************************************************************************/
/*!
    @brief  Calculate the roll, pitch and yaw in degrees
*/
/******************************************************************************/
axesData_t Attitude_Determination::getAngles()
{
    if ( !_anglesComputed )
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
    axesData_t tmpSpn;
    axesData_t tmpAtt = {_attitude.a, _attitude.b, _attitude.c};
    float dtSec = (float) dt / 1000;
    dataPacket_t heading;
    sensors_event_t eventG;
    _gyro.getEvent(&eventG);
    _spin.x = eventG.gyro.x - _bias.x;
    _spin.y = eventG.gyro.y - _bias.y;
    _spin.z = eventG.gyro.z - _bias.z;
    heading.spin = _spin;
    /* Integrate rate of change of quaternions */
    tmpSpn.x = _spin.x * (0.5f * dtSec);
    tmpSpn.y = _spin.y * (0.5f * dtSec);
    tmpSpn.z = _spin.z * (0.5f * dtSec);
    _attitude.a += (-tmpAtt.y * tmpSpn.x - tmpAtt.z * tmpSpn.y - _attitude.d * tmpSpn.z);
    _attitude.b += ( tmpAtt.x * tmpSpn.x + tmpAtt.z * tmpSpn.z - _attitude.d * tmpSpn.y);
    _attitude.c += ( tmpAtt.x * tmpSpn.y - tmpAtt.y * tmpSpn.z - _attitude.d * tmpSpn.x);
    _attitude.d += ( tmpAtt.x * tmpSpn.z + tmpAtt.y * tmpSpn.y -     tmpAtt.z * tmpSpn.x);
    /* Normalize the quaternions */
    normal = inverseSqrt(_attitude.a * _attitude.a +
                         _attitude.b * _attitude.b +
                         _attitude.c * _attitude.c +
                         _attitude.d * _attitude.d );
    _attitude.a *= normal;
    _attitude.b *= normal;
    _attitude.c *= normal;
    _attitude.d *= normal;
    heading.attitude = _attitude;
    _anglesComputed = false;

    return heading;
}