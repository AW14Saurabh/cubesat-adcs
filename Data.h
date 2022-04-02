/*******************************************************************************
 * This library defines all the data structures used.
 ******************************************************************************/

/* States */
#define DISABLED 0
#define ENABLED 1
#define DETUMBLE 0
#define POINT 1

/*==============================================================================
    DATA TYPE FOR AXES
    --------------------------------------------------------------------------*/
template <typename T>
struct axesData_s
{
    T x;
    T y;
    T z;
};
typedef axesData_s<float> angVelData_t, angRPYData_t, angMomData_t;
typedef axesData_s<int>   motFreqData_t;
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
    DATA TYPE FOR MESSAGE PACKET
    --------------------------------------------------------------------------*/
typedef struct messageData_s
{
    int enableState;
    int mode;
    float targetAngle;
} messageData_t;
/*============================================================================*/
