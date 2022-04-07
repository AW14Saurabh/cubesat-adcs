#include "Attitude_Determination.h"
#include "Radio_Communication.h"
#include "Motor_Control.h"

#define BEEPER 9
#define LASER 10

Attitude_Determination *attitude;
Radio_Communication *radio;
Motor_Control *motors;

angRPYData_t angles;
dataPacket_t heading;
messageData_t message;

int32_t dt = 0;
uint64_t previousMillis = 0ul;
uint64_t currentMillis = 0ul;

void setup()
{
    pinMode(BEEPER, OUTPUT);
    pinMode(LASER,  OUTPUT);
    motors = new Motor_Control();
    radio = new Radio_Communication();
    attitude = new Attitude_Determination();
    digitalWrite(BEEPER, HIGH);
    delay(10);
    digitalWrite(BEEPER, LOW);
}

void loop()
{
    currentMillis = millis();
    dt = currentMillis - previousMillis;

    message = radio->getMessage();

    digitalWrite(LASER, message.laserEnable);

    if (dt >= MIN_SAMPLE_TIME)
    {
        heading = attitude->updateHeading(dt);
        angles = attitude->getAngles(); //To Serial
    }

    motors->updateMotor(heading, message, dt);
    radio->sendMessage(&angles);
    previousMillis = currentMillis;
}
