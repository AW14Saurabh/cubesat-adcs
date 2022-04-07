/*******************************************************************************
 * This library defines all the data structures used.
 ******************************************************************************/
#ifndef __DATA_H__
#define __DATA_H__

#include<Arduino.h>
/* States */
#define DETUMBLE 0
#define POINT 1

/*==============================================================================
    DATA TYPE FOR AXES
    --------------------------------------------------------------------------*/
typedef struct axesData_s
{
    float x;
    float y;
    float z;
} angVelData_t, angRPYData_t, angMomData_t;
/*============================================================================*/

/*==============================================================================
    DATA TYPE FOR QUATERNION
    --------------------------------------------------------------------------*/
typedef struct quaternionData_s
{
    float a;
    float b;
    float c;
    float d;
} attdData_t;
/*============================================================================*/

/*==============================================================================
    DATA TYPE FOR GYRO EVENT
    --------------------------------------------------------------------------*/
typedef struct dataPacket_s
{
    angVelData_t angVel;
    attdData_t attitude;
} dataPacket_t;
/*============================================================================*/

/*==============================================================================
    DATA TYPE FOR MESSAGE PACKET FROM CONTROL BOX TO SATELLITE
    --------------------------------------------------------------------------*/
typedef struct messageData_s
{
    int laserEnable;
    int opMode;
    angRPYData_t targetAngles;
} messageData_t;
/*============================================================================*/
#endif
