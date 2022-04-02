/*******************************************************************************
 * This library is an interface to the 3 reaction wheel motors. Calculates error
 * and adjust motor for detumbling and point functions.
 ******************************************************************************/
#ifndef __MOTOR_CONTROL_H__
#define __MOTOR_CONTROL_H__

#include "Data.h"

#define MOT1_SPD 2
#define MOT1_DIR 4
#define MOT1_FREQ 3 // PWM

#define MOT2_SPD 14
#define MOT2_DIR 15
#define MOT2_FREQ 5 // PWM

#define MOT3_SPD 16
#define MOT3_DIR 17
#define MOT3_FREQ 6 // PWM

#define MOTOR_MAX_ACCEL 248.8
#define WHEEL_I 1.41E-05
#define MAX_MOTOR_W 1376
#define MOTOR_MIDDLE_SPEED 754

class Motor_Control
{
private:
    angVelData_t  _satAngVel;      //3 Axis Rotation, 1 Satellite
    attdData_t    _satAttitude;
    angMomData_t  _satAngMom;
    motFreqData_t _motorFreq;
    float         _wheelAngVel[3]; //Single Axis Rotation, 3 Wheels
    float           _dt;           //seconds interval
    void computeAngles(angRPYData_t *);
    void detumble();
    void point(float);
    void calcWheelAngVel();
    void setMotor();

public:
    Motor_Control();
    void updateMotor(dataPacket_t, messageData_t, int);
    motFreqData_t getFrequency();
};

#endif