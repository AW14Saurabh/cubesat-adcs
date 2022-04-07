/*******************************************************************************
 * This library is an interface to the 3 reaction wheel motors. Calculates error
 * and adjust motor for detumbling and point functions.
 ******************************************************************************/
#include "Motor_Control.h"

/*******************************************************************************
 PRIVATE FUNCTIONS
 ******************************************************************************/

/******************************************************************************/
/*!
    @brief  Compute angle in radians from the quaternions
*/
/******************************************************************************/
void Motor_Control::computeAngles()
{
    _satAngles.x = atan2f(_satAttitude.a * _satAttitude.b +
                              _satAttitude.c * _satAttitude.d,
                          0.5f -
                              _satAttitude.b * _satAttitude.b -
                              _satAttitude.c * _satAttitude.c);

    _satAngles.y = asinf(-2.0f *
                         (_satAttitude.b * _satAttitude.d -
                          _satAttitude.a * _satAttitude.c));

    _satAngles.z = atan2f(_satAttitude.b * _satAttitude.c +
                              _satAttitude.a * _satAttitude.d,
                          0.5f -
                              _satAttitude.c * _satAttitude.c -
                              _satAttitude.d * _satAttitude.d);
}

/******************************************************************************/
/*!
    @brief  Calculate combined angular momentum to be generated by all wheels
*/
/******************************************************************************/
void Motor_Control::detumble()
{
    float P = -0.003;
    _satAngMom.x = P * (0 - _satAngVel.x) * _dt;
    _satAngMom.y = P * (0 - _satAngVel.y) * _dt;
    _satAngMom.z = P * (0 - _satAngVel.z) * _dt;
}

/******************************************************************************/
/*!
    @brief  Calculate combined angular momentum required from all wheels,
            only affects yaw angle.
*/
/******************************************************************************/
void Motor_Control::point(angRPYData_t targetAng)
{
    angRPYData_t error;
    computeAngles();

    error.x = targetAng.x * PI / 180 - _satAngles.x;
    error.y = targetAng.y * PI / 180 - _satAngles.y;
    error.z = targetAng.z * PI / 180 - _satAngles.z;

    while (error.z  >  PI) error.z -= 2 * PI;
    while (error.z <= -PI) error.z += 2 * PI;

    float P = -0.001;
    float D =  0.003;
    _satAngMom.x = (P * error.x + D * _satAngVel.x) * _dt;
    _satAngMom.y = (P * error.y + D * _satAngVel.y) * _dt;
    _satAngMom.z = (P * error.z + D * _satAngVel.z) * _dt;
}

/******************************************************************************/
/*!
    @brief  Distribute combined angular momentum to each wheel, calculate
            scaled angular velocity for each wheel.
*/
/******************************************************************************/
void Motor_Control::calcWheelAngVel()
{
    /*
    Convert satellite angular momentum to wheel angular velocity change and add 
    to current wheel angular velocity. Since motors are on the axes there is no
    complex momentum distribution math involved.
    */
    float wheelAngVelDelta[3];
    wheelAngVelDelta[0] = _satAngMom.x / WHEEL_I;
    wheelAngVelDelta[1] = _satAngMom.y / WHEEL_I;
    wheelAngVelDelta[2] = _satAngMom.z / WHEEL_I;

    /* Scale down angVelDelta if accel is too large */
    float maxAngVelDelta = 0;
    for (int i = 0; i < 3; i++)
        maxAngVelDelta = max(abs(wheelAngVelDelta[i]), maxAngVelDelta);
    float maxVelDeltaAllowed = MOTOR_MAX_ACCEL * _dt;
    if (maxAngVelDelta > maxVelDeltaAllowed)
    {
        wheelAngVelDelta[0] *= maxVelDeltaAllowed / maxAngVelDelta;
        wheelAngVelDelta[1] *= maxVelDeltaAllowed / maxAngVelDelta;
        wheelAngVelDelta[2] *= maxVelDeltaAllowed / maxAngVelDelta;
    }

    /* If motors are saturated, don't update */
    float maxAngVel = 0;
    for (int i = 0; i < 3; i++)
        maxAngVel = max(abs(_wheelAngVel[i] + wheelAngVelDelta[i]), maxAngVel);
    if (maxAngVel > MAX_MOTOR_W)
    {
        wheelAngVelDelta[0] = 0;
        wheelAngVelDelta[1] = 0;
        wheelAngVelDelta[2] = 0;
    }

    _wheelAngVel[0] += wheelAngVelDelta[0];
    _wheelAngVel[1] += wheelAngVelDelta[1];
    _wheelAngVel[2] += wheelAngVelDelta[2];
}

/******************************************************************************/
/*!
    @brief  Convert wheel angular velocity to PWM signal and send to the motors.
*/
/******************************************************************************/
void Motor_Control::setMotor()
{
    bool dir[3];
    float spd[3];
    for (int i = 0; i < 3; i++)
    {
        dir[i] = _wheelAngVel[i] < 0; //Change < to >= to reverse the working.
        spd[i] = map(abs(_wheelAngVel[i]), 0, MAX_MOTOR_W, 0, MAX_MOTOR_PWM);
    }

    digitalWrite(MOT1_IN1,  dir[0]);
    digitalWrite(MOT2_IN1, !dir[0]);
    digitalWrite(MOT3_IN1,  dir[1]);
    digitalWrite(MOT1_IN2, !dir[1]);
    digitalWrite(MOT2_IN2,  dir[2]);
    digitalWrite(MOT3_IN2, !dir[2]);
     analogWrite(MOT1_SPD,  spd[0]);
     analogWrite(MOT2_SPD,  spd[1]);
     analogWrite(MOT3_SPD,  spd[2]);
}

/*******************************************************************************
 CONSTRUCTOR
 ******************************************************************************/

/******************************************************************************/
/*!
    @brief  Instantiates a new Motor_Control class
*/
/******************************************************************************/
Motor_Control::Motor_Control() : _satAngVel{0,0,0},
                                 _satAttitude{0,0,0,0},
                                 _satAngMom{0,0,0},
                                 _satAngles{0,0,0},
                                 _wheelAngVel{0,0,0},
                                 _dt (0)
{
    pinMode(MOT1_IN1, OUTPUT);
    pinMode(MOT1_IN2, OUTPUT);
    pinMode(MOT1_SPD, OUTPUT);
    pinMode(MOT2_IN1, OUTPUT);
    pinMode(MOT2_IN2, OUTPUT);
    pinMode(MOT2_SPD, OUTPUT);
    pinMode(MOT3_IN1, OUTPUT);
    pinMode(MOT3_IN2, OUTPUT);
    pinMode(MOT3_SPD, OUTPUT);

    digitalWrite(MOT1_IN1,LOW);
    digitalWrite(MOT2_IN1,LOW);
    digitalWrite(MOT3_IN1,LOW);
    digitalWrite(MOT1_IN2,HIGH);
    digitalWrite(MOT2_IN2,HIGH);
    digitalWrite(MOT3_IN2,HIGH);
    analogWrite(MOT1_SPD, MAX_MOTOR_PWM);
    analogWrite(MOT2_SPD, MAX_MOTOR_PWM);
    analogWrite(MOT3_SPD, MAX_MOTOR_PWM);
}

/*******************************************************************************
 PUBLIC FUNCTIONS
 ******************************************************************************/

/******************************************************************************/
/*!
    @brief  Update motors from gathered data
*/
/******************************************************************************/
void Motor_Control::updateMotor(dataPacket_t heading, messageData_t message, int dtMillis)
{
    _satAngVel = heading.angVel;
    _satAttitude = heading.attitude;
    _dt = (float) dtMillis / 1000.0;

    if (message.opMode == DETUMBLE)
        detumble();
    else if (message.opMode == POINT)
        point(message.targetAngles);
    calcWheelAngVel();
    setMotor();
}
