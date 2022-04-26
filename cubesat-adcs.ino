#include "Attitude_Determination.h"
#include "Radio_Communication.h"
#include "Motor_Control.h"

#define BEEPER 9
#define LASER 10

Attitude_Determination *attitude;
Radio_Communication *radio;
Motor_Control *motors;

messageData_t message {1, 0, {0,0,0}};
angVelData_t satAngVel {0,0,0};
attdData_t satAttitude {1,0,0,0};
angRPYData_t angles {0,0,0};

int32_t dt = 0;
uint64_t previousMillis = 0ul;
uint64_t currentMillis = 0ul;

void setup()
{
    Serial.begin(9600);
    pinMode(BEEPER, OUTPUT);
    pinMode(LASER,  OUTPUT);
    analogWrite(LASER, 200);
    motors = new Motor_Control(&satAngVel, &angles);
    radio = new Radio_Communication();
    attitude = new Attitude_Determination();
    tone(BEEPER, 5000, 500);
    analogWrite(LASER, 0);
}

void loop()
{
    currentMillis = millis();
    dt = currentMillis - previousMillis;

    radio->getMessage(&message);

    analogWrite(LASER, !message.laserDisable * 200);

    if (dt >= MIN_SAMPLE_TIME)
    {
        attitude->updateHeading(&satAngVel, &satAttitude, dt);
        attitude->getAngles(&angles, &satAttitude); //To Serial
    }

    motors->updateMotor(&message, dt);
    radio->sendMessage(&angles);

    previousMillis = currentMillis;
}
