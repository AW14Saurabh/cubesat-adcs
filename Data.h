/*******************************************************************************
 * This library defines all the data structures used.
 ******************************************************************************/
#ifndef __DATA_H__
#define __DATA_H__

#include<Arduino.h>
/* States */
#define DETUMBLE 1
#define POINT 0

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
    DATA TYPE FOR MESSAGE PACKET FROM CONTROL BOX TO SATELLITE
    --------------------------------------------------------------------------*/
typedef struct messageData_s
{
    bool laserDisable;
    bool opMode;
    angRPYData_t targetAngles;
} messageData_t;
/*============================================================================*/
#endif
