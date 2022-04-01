#include "Attitude_Determination.h"
#include "Radio_Communication.h"

Attitude_Determination *attitude;
Radio_Communication *radio;

axesData_t angles;
dataPacket_t heading;
messageData_t message;

int32_t dt = 0;
uint64_t previousMillis = 0ul;
uint64_t currentMillis = 0ul;

void setup()
{
    attitude = new Attitude_Determination();
    radio = new Radio_Communication();
}

void loop()
{
    currentMillis = millis();
    dt = currentMillis - previousMillis;

    message = radio->getMessage();

    if (dt >= MIN_SAMPLE_TIME)
    {
        heading = attitude->updateHeading(dt);
        angles = attitude->getAngles();
    }
}
