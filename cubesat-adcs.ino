#include "Attitude_Determination.h"
#include "Radio_Communication.h"
#include "Motor_Control.h"

#define BEEPER 9
#define LASER 10

Attitude_Determination *attitude;
Radio_Communication *radio;
Motor_Control *motors;

angRPYData_t angles {0,0,0};
dataPacket_t heading {{0,0,0},{1,0,0,0}};
messageData_t message {0, 0, {0,0,0}};

int32_t dt = 0;
uint64_t previousMillis = 0ul;
uint64_t currentMillis = 0ul;

void setup()
{
    Serial.begin(9600);
    pinMode(BEEPER, OUTPUT);
    pinMode(LASER,  OUTPUT);
    analogWrite(LASER, 200);
    motors = new Motor_Control();
    radio = new Radio_Communication();
    attitude = new Attitude_Determination();
    tone(BEEPER, 5000, 5000);
    analogWrite(LASER, 0);
}

void loop()
{
    currentMillis = millis();
    dt = currentMillis - previousMillis;

    message = radio->getMessage();
    Serial.println(String("Laser: ") + String(message.laserEnable ? "Off" : "On"));
    Serial.println(String("Satellite Operation: ") + String(message.opMode) + String(message.opMode ? " Detumbling" : " Pointing"));

    Serial.println("Roll: " + String(message.targetAngles.x) + "\tPitch: " + String(message.targetAngles.y) + "\tYaw: " + String(message.targetAngles.z));
    /*
    // digitalWrite(LASER, message.laserEnable);

    if (dt >= MIN_SAMPLE_TIME)
    {
        heading = attitude->updateHeading(dt);
        angles = attitude->getAngles(); //To Serial
    }

    motors->updateMotor(heading, message, dt);
    */
    radio->sendMessage(&message.targetAngles);
    previousMillis = currentMillis;
}
