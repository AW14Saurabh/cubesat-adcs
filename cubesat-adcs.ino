#include "Attitude_Determination.h"
#include "Radio_Communication.h"
#include "Motor_Control.h"

#define BEEPER 9
#define LASER 10

Attitude_Determination *attitude;
Radio_Communication *radio;
Motor_Control *motors;

messageData_t message {1, 0, {0,0,0}};
angVelData_t satAngVel {0.0,0.0,0.0};
attdData_t satAttitude {1.0,0.0,0.0,0.0};
angRPYData_t angles {0.0,0.0,0.0};

int32_t dt = 0;
uint64_t previousMillis = 0ul;
uint64_t currentMillis = 0ul;

void setup()
{
    // Serial.begin(115200);
    pinMode(BEEPER, OUTPUT);
    pinMode(LASER,  OUTPUT);
    analogWrite(LASER, 200);
    // Serial.println("Radio On");
    motors = new Motor_Control(&satAngVel, &angles);
    // Serial.println("Motors On");
    attitude = new Attitude_Determination();
    radio = new Radio_Communication();
    // Serial.println("Gyro On");
    tone(BEEPER, 5000, 500);
    analogWrite(LASER, 0);
    // delay(200);
}

void loop()
{
    currentMillis = millis();
    dt = currentMillis - previousMillis;

    radio->getMessage(&message);
    // Serial.print("Operation: " + String(message.opMode?"D":"P") + " Laser: " + String(message.laserDisable?"Off ":"On  "));
    // Serial.println("Target Angle: " + String(message.targetAngles.z));
    // delay(100);
    analogWrite(LASER, !message.laserDisable * 200);

    attitude->updateHeading(&satAngVel, &satAttitude, dt);
    // Serial.println("AngVel: " + String(satAngVel.x, 4) + " " + String(satAngVel.y, 4) + " " + String(satAngVel.z, 4));
    // Serial.println("Attd: " + String(satAttitude.a) + " " + String(satAttitude.b) + " " + String(satAttitude.c) + " " + String(satAttitude.d));
    // attitude->getAngles(&angles, &satAttitude); //To Serial
    // Serial.println("Angles: " + String(angles.x * 57.29578f) + " " + String(angles.y * 57.29578f) + " " + String(angles.z * 57.29578f));

    motors->updateMotor(&message, dt);
    // Serial.println("Motors Updated");
    radio->sendMessage(&satAngVel);

    previousMillis = currentMillis;
    delay(100);
}
