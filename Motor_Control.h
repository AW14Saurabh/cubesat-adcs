/*******************************************************************************
 * This library is an interface to the 3 reaction wheel motors. Calculates error
 * and adjust motor for detumbling and point functions.
 ******************************************************************************/
#ifndef __MOTOR_CONTROL_H__
#define __MOTOR_CONTROL_H__

#include "Data.h"

#define MOT_Y_IN1 2
#define MOT_Y_IN2 4
#define MOT_Y_SPD 3 // PWM

#define MOT_R_IN1 14
#define MOT_R_IN2 15
#define MOT_R_SPD 5 // PWM

#define MOT_P_IN1 16
#define MOT_P_IN2 17
#define MOT_P_SPD 6 // PWM

#define MOTOR_MAX_ACCEL 134.74
#define WHEEL_I 2.835E-05 //N.m
#define MAX_MOTOR_W 2400
#define MAX_MOTOR_PWM 255

class Motor_Control
{
private:
    angVelData_t* _satAngVel;      //3 Axis Rotation, 1 Satellite
    angMomData_t  _satAngMom;
    angRPYData_t* _satAngles;
    float         _wheelAngVel[3]; //Single Axis Rotation, 3 Wheels
    void detumble();
    void point(angRPYData_t*);
    void calcWheelAngVel();
    void setMotor();

public:
    Motor_Control(angVelData_t*, angRPYData_t*);
    void updateMotor(messageData_t*);
};

#endif