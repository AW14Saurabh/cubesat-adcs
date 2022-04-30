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
                                                   _mag(Adafruit_LSM303_Mag_Unified(30302))
{
    /* Initialize IMU */
    cal.begin();
    cal.loadCalibration();
    _gyro.enableAutoRange(true);
    _accel.enableAutoRange(true);
    _mag.enableAutoRange(true);
    _gyro.begin();
    _accel.begin();
    _mag.begin();
    filter.begin(FILTER_UPDATE_RATE_HZ);
}

/*******************************************************************************
 PUBLIC FUNCTIONS
 ******************************************************************************/

/******************************************************************************/
/*!
    @brief  Update the Heading from gyro reading
*/
/******************************************************************************/
void Attitude_Determination::updateHeading(angVelData_t *w, angRPYData_t *ang, attdData_t *q)
{
    sensors_event_t eventG, eventA, eventM;

    _accel.getEvent(&eventA);
    _mag.getEvent(&eventM);
    _gyro.getEvent(&eventG);

    cal.calibrate(eventA);
    cal.calibrate(eventM);
    cal.calibrate(eventG);

    w->x = eventG.gyro.x * SENSORS_RADS_TO_DPS;
    w->y = eventG.gyro.y * SENSORS_RADS_TO_DPS;
    w->z = eventG.gyro.z * SENSORS_RADS_TO_DPS;

    filter.update(w->x, w->y, w->z,
                  eventA.acceleration.x, eventA.acceleration.y, eventA.acceleration.z,
                  eventM.magnetic.x, eventM.magnetic.y, eventM.magnetic.z);

    ang->x += filter.getRoll();
    ang->y += filter.getPitch();
    ang->z += filter.getYaw();

    filter.getQuaternion(&q->a, &q->b, &q->c, &q->d);
}