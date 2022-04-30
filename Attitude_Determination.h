/*******************************************************************************
 * This library is an interface to the L3GD20 Gyroscope connected to the board
 * on Analog pins Pin 18 A4 SDA and Pin 19 A5 SCL on the I2C bus using the
 * SoftI2C library.
 ******************************************************************************/
#ifndef __ATTITUDE_DETERMINATION_H__
#define __ATTITUDE_DETERMINATION_H__

#define ID 20
#define MIN_SAMPLE_TIME 30
#define FILTER_UPDATE_RATE_HZ 100

#define __AVR_ATmega328P__
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_Sensor_Calibration.h>
#include <Adafruit_AHRS.h>
#include "Data.h"

class Attitude_Determination
{
private:
    Adafruit_LSM303_Accel_Unified _accel;
    Adafruit_LSM303_Mag_Unified _mag;
    Adafruit_L3GD20_Unified _gyro;
    Adafruit_Mahony filter;
    Adafruit_Sensor_Calibration_EEPROM cal;

public:
    Attitude_Determination();

    void updateHeading(angVelData_t*, angRPYData_t*, attdData_t*);
};

#endif