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

void setup()
{
    Serial.begin(115200);
    pinMode(BEEPER, OUTPUT);
    pinMode(LASER,  OUTPUT);
    analogWrite(LASER, 200);
    // motors = new Motor_Control(&satAngVel, &angles);
    attitude = new Attitude_Determination();
    // radio = new Radio_Communication();
    tone(BEEPER, 5000, 500);
    analogWrite(LASER, 0);
}

void loop()
{
    // radio->getMessage(&message);
    // Serial.print("Operation: " + String(message.opMode?"D":"P") + " Laser: " + String(message.laserDisable?"Off ":"On  "));
    // Serial.println("Target Angle: " + String(message.targetAngles.z));
    // analogWrite(LASER, !message.laserDisable * 200);

    attitude->updateHeading(&satAngVel, &angles, &satAttitude);
    Serial.println("AngVel   :\t" + String(satAngVel.x, 6) + "\t" + String(satAngVel.y, 6) + "\t" + String(satAngVel.z, 6));
    // Serial.println("Attd     :\t" + String(satAttitude.a, 6) + "\t" + String(satAttitude.b, 6) + "\t" + String(satAttitude.c, 6) + "\t" + String(satAttitude.d));
    // Serial.println("AnglesDeg:\t" + String(angles.x, 6) + "\t" + String(angles.y, 6) + "\t" + String(angles.z, 6));

    // motors->updateMotor(&message);
    // radio->sendMessage(&satAngVel);

    delay(10);
}
