/*******************************************************************************
 * This library is an interface to the L3GD20 Gyroscope connected to the board
 * on Analog pins Pin 18 A4 SDA and Pin 19 A5 SCL on the I2C bus using the
 * SoftI2C library.
 ******************************************************************************/
#include "Attitude_Determination.h"

/*******************************************************************************
 CONSTRUCTOR
 ******************************************************************************/

/******************************************************************************/
/*!
    @brief  Instantiates a new Attitude_Determination class
*/
/******************************************************************************/
Attitude_Determination::Attitude_Determination() : _gyro(Adafruit_L3GD20_Unified(ID)),
                                                   _accel(Adafruit_LSM303_Accel_Unified(30301)),
                                                   _mag(Adafruit_LSM303_Mag_Unified(30302)),
                                                   _bias({0.0,0.0,0.0})
{
    /* Initialize IMU */
    cal.begin();
    cal.loadCalibration();
    // _gyro.enableAutoRange(true));
    // _accel.enableAutoRange(true);
    // _mag.enableAutoRange(true);
    _mag.setMagGain(LSM303_MAGGAIN_4_0);
    _gyro.begin(GYRO_RANGE_2000DPS);
    _accel.begin();
    _mag.begin();
    filter.begin(FILTER_UPDATE_RATE_HZ);
    for(int i = 0; i < 500; i++)
    {
        sensors_event_t event;
        _gyro.getEvent(&event);
        _bias.x += event.gyro.x;
        _bias.y += event.gyro.y;
        _bias.z += event.gyro.z;
        delay(10);
    }
    _bias.x /= 500;
    _bias.y /= 500;
    _bias.z /= 500;
}

/*******************************************************************************
 PUBLIC FUNCTIONS
 ******************************************************************************/

/******************************************************************************/
/*!
    @brief  Update the Heading from gyro reading
*/
/******************************************************************************/
void Attitude_Determination::updateHeading(angVelData_t *w /*, angRPYData_t *ang, attdData_t *q*/)
{
    sensors_event_t eventG, eventA, eventM;

    _accel.getEvent(&eventA);
    _mag.getEvent(&eventM);
    _gyro.getEvent(&eventG);

    cal.calibrate(eventA);
    cal.calibrate(eventM);
    cal.calibrate(eventG);

    w->x = eventG.gyro.x - _bias.x;
    w->y = eventG.gyro.y - _bias.y;
    w->z = eventG.gyro.z - _bias.z;

    filter.update(w->x * SENSORS_RADS_TO_DPS, w->y * SENSORS_RADS_TO_DPS, w->z * SENSORS_RADS_TO_DPS,
                  eventA.acceleration.x, eventA.acceleration.y, eventA.acceleration.z,
                  eventM.magnetic.x, eventM.magnetic.y, eventM.magnetic.z);

    // ang->x = filter.getRoll();
    // ang->y = filter.getPitch();
    // ang->z = filter.getYaw();
    // ang->x += w->x * 0.1;
    // ang->y += w->y * 0.1;
    // ang->z += w->z * 0.1;

    // filter.getQuaternion(&q->a, &q->b, &q->c, &q->d);
}