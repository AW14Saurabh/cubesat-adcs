#include "Attitude_Determination.h"

#define MIN_SAMPLE_TIME 30

Attitude_Determination attitude;
axesData_t angles;
dataPacket_t heading;

int32_t dt = 0;
uint64_t previousMillis = 0ul;
uint64_t currentMillis = 0ul;

void setup()
{
    attitude = Attitude_Determination();
}

void loop()
{
    currentMillis = millis();
    dt = currentMillis - previousMillis;
    if (dt >= MIN_SAMPLE_TIME)
    {
        heading = attitude.updateHeading(dt);
        angles = attitude.getAngles();
    }
}
