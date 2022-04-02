#include "Attitude_Determination.h"
#include "Radio_Communication.h"
#include "Motor_Control.h"

#define BEEPER 7
#define STATUS_LED 8

Attitude_Determination *attitude;
Radio_Communication *radio;
Motor_Control *motors;

angRPYData_t angles;
dataPacket_t heading;
messageData_t message;
motFreqData_t frequency;

int32_t dt = 0;
uint64_t previousMillis = 0ul;
uint64_t currentMillis = 0ul;

void setup()
{
    attitude = new Attitude_Determination();
    radio = new Radio_Communication();
    motors = new Motor_Control();
}

void loop()
{
    currentMillis = millis();
    dt = currentMillis - previousMillis;

    message = radio->getMessage();

    if (dt >= MIN_SAMPLE_TIME)
    {
        heading = attitude->updateHeading(dt);
        angles = attitude->getAngles(); //To Serial
    }

    motors->updateMotor(heading, message, dt);
    frequency = motors->getFrequency();

    previousMillis = currentMillis;
}
