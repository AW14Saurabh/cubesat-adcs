/*******************************************************************************
 * This library is an interface to the 3 reaction wheel motors. Calculates error
 * and adjust motor for detumbling and point functions.
 ******************************************************************************/
#ifndef __MOTOR_CONTROL_H__
#define __MOTOR_CONTROL_H__

#include "Data.h"

#define MOT1_IN1 2
#define MOT1_IN2 4
#define MOT1_SPD 3 // PWM

#define MOT2_IN1 14
#define MOT2_IN2 15
#define MOT2_SPD 5 // PWM

#define MOT3_IN1 16
#define MOT3_IN2 17
#define MOT3_SPD 6 // PWM

#define MOTOR_MAX_ACCEL 134.74
#define WHEEL_I 2.835E-05
#define MAX_MOTOR_W 2970
#define MAX_MOTOR_PWM 180

class Motor_Control
{
private:
    angVelData_t  _satAngVel;      //3 Axis Rotation, 1 Satellite
    attdData_t    _satAttitude;
    angMomData_t  _satAngMom;
    angRPYData_t  _satAngles;
    float         _wheelAngVel[3]; //Single Axis Rotation, 3 Wheels
    float           _dt;           //seconds interval
    void computeAngles();
    void detumble();
    void point(angRPYData_t);
    void calcWheelAngVel();
    void setMotor();

public:
    Motor_Control();
    void updateMotor(dataPacket_t, messageData_t, int);
};

#endif