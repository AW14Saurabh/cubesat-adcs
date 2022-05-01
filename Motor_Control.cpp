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
    @brief  Calculate combined angular momentum to be generated by all wheels
*/
/******************************************************************************/
void Motor_Control::detumble()
{
    float P = 1.0;
    _satAngMom.x = P * (0.0 - _satAngVel->x);
    _satAngMom.y = P * (0.0 - _satAngVel->y);
    _satAngMom.z = P * (0.0 - _satAngVel->z);
}

/******************************************************************************/
/*!
    @brief  Calculate combined angular momentum required from all wheels,
            only affects yaw angle. Target Angles are in degrees.
*/
/******************************************************************************/
// void Motor_Control::point(angRPYData_t *targetAng)
// {
//     angRPYData_t error;

//     error.x = _satAngles->x - targetAng->x;
//     error.y = _satAngles->y - targetAng->y;
//     error.z = _satAngles->z - targetAng->z;

//     float P = 2.25;
//     float D = 2.0;
//     _satAngMom.x = P * error.x + D * _satAngVel->x;
//     _satAngMom.y = P * error.y + D * _satAngVel->y;
//     _satAngMom.z = P * error.z + D * _satAngVel->z;
// }

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
    float whlAngVelD[3];
    whlAngVelD[0] = _satAngMom.x / WHEEL_I;
    whlAngVelD[1] = _satAngMom.y / WHEEL_I;
    whlAngVelD[2] = _satAngMom.z / WHEEL_I;

    /* Scale down angVelDelta if accel is too large */
    
    whlAngVelD[0] = copysign(min(abs(whlAngVelD[0]), MOTOR_MAX_ACCEL * 0.1), whlAngVelD[0]);
    whlAngVelD[1] = copysign(min(abs(whlAngVelD[1]), MOTOR_MAX_ACCEL * 0.1), whlAngVelD[1]);
    whlAngVelD[2] = copysign(min(abs(whlAngVelD[2]), MOTOR_MAX_ACCEL * 0.1), whlAngVelD[2]);

    /* If motors are saturated, don't update */
    whlAngVelD[0] = copysign(min(abs(whlAngVelD[0] + _whlAngVel[0]), MAX_MOTOR_W) - abs(_whlAngVel[0]), whlAngVelD[0]);
    whlAngVelD[1] = copysign(min(abs(whlAngVelD[1] + _whlAngVel[1]), MAX_MOTOR_W) - abs(_whlAngVel[1]), whlAngVelD[1]);
    whlAngVelD[2] = copysign(min(abs(whlAngVelD[2] + _whlAngVel[2]), MAX_MOTOR_W) - abs(_whlAngVel[2]), whlAngVelD[2]);

    _whlAngVel[0] += whlAngVelD[0];
    _whlAngVel[1] += whlAngVelD[1];
    _whlAngVel[2] += whlAngVelD[2];
    Serial.println("wheelW: " + String(_whlAngVel[0], 2) + " " + String(_whlAngVel[1], 2) + " " + String(_whlAngVel[2], 2));
}

/******************************************************************************/
/*!
    @brief  Convert wheel angular velocity to PWM signal and send to the motors.
*/
/******************************************************************************/
void Motor_Control::setMotor()
{
    digitalWrite(MOT_R_IN1, _whlAngVel[0] < 0 ? LOW : HIGH);
    digitalWrite(MOT_R_IN2, _whlAngVel[0] < 0 ? HIGH : LOW);
    digitalWrite(MOT_P_IN1, _whlAngVel[1] < 0 ? LOW : HIGH);
    digitalWrite(MOT_P_IN2, _whlAngVel[1] < 0 ? HIGH : LOW);
    digitalWrite(MOT_Y_IN1, _whlAngVel[2] < 0 ? LOW : HIGH);
    digitalWrite(MOT_Y_IN2, _whlAngVel[2] < 0 ? HIGH : LOW);
    analogWrite(MOT_R_SPD, map(abs(_whlAngVel[0]), 0, MAX_MOTOR_W, 0, MAX_MOTOR_PWM));
    analogWrite(MOT_P_SPD, map(abs(_whlAngVel[1]), 0, MAX_MOTOR_W, 0, MAX_MOTOR_PWM));
    analogWrite(MOT_Y_SPD, map(abs(_whlAngVel[2]), 0, MAX_MOTOR_W, 0, MAX_MOTOR_PWM));
    
}

/*******************************************************************************
 CONSTRUCTOR
 ******************************************************************************/

/******************************************************************************/
/*!
    @brief  Instantiates a new Motor_Control class
*/
/******************************************************************************/
Motor_Control::Motor_Control(angVelData_t *angVel /*, angRPYData_t *ang */) : _satAngVel(angVel),
                                                                        // _satAngles(ang),
                                                                        _satAngMom{0,0,0},
                                                                        _whlAngVel{0,0,0}
{
    pinMode(MOT_Y_IN1, OUTPUT);
    pinMode(MOT_Y_IN2, OUTPUT);
    pinMode(MOT_Y_SPD, OUTPUT);
    pinMode(MOT_R_IN1, OUTPUT);
    pinMode(MOT_R_IN2, OUTPUT);
    pinMode(MOT_R_SPD, OUTPUT);
    pinMode(MOT_P_IN1, OUTPUT);
    pinMode(MOT_P_IN2, OUTPUT);
    pinMode(MOT_P_SPD, OUTPUT);

    digitalWrite(MOT_Y_IN1,LOW);
    digitalWrite(MOT_R_IN1,LOW);
    digitalWrite(MOT_P_IN1,LOW);
    digitalWrite(MOT_Y_IN2,HIGH);
    digitalWrite(MOT_R_IN2,HIGH);
    digitalWrite(MOT_P_IN2,HIGH);
    analogWrite(MOT_Y_SPD, 180);
    analogWrite(MOT_R_SPD, 180);
    analogWrite(MOT_P_SPD, 180);
}

/*******************************************************************************
 PUBLIC FUNCTIONS
 ******************************************************************************/

/******************************************************************************/
/*!
    @brief  Update motors from gathered data
*/
/******************************************************************************/
void Motor_Control::updateMotor(messageData_t *message)
{
    // if (message->opMode)
    detumble();
    // else
    // point(&message->targetAngles);
    calcWheelAngVel();
    setMotor();
}
