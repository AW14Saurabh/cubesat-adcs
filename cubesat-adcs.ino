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
// attdData_t satAttitude {1.0,0.0,0.0,0.0};
// angRPYData_t angles {0.0,0.0,0.0};

void setup()
{
    Serial.begin(115200);
    pinMode(BEEPER, OUTPUT);
    pinMode(LASER,  OUTPUT);
    analogWrite(LASER, 200);
    attitude = new Attitude_Determination();
    Serial.println("Gyro On");
    motors = new Motor_Control(&satAngVel/*, &angles*/);
    Serial.println("Motors On");
    radio = new Radio_Communication();
    Serial.println("Radio On");
    tone(BEEPER, 5000, 500);
    analogWrite(LASER, 0);
}

void loop()
{
    radio->getMessage(&message);
    // Serial.print("Operation: " + String(message.opMode?"D":"P") + " Laser: " + String(message.laserDisable?"Off ":"On  "));
    // Serial.println("Target Angle: " + String(message.targetAngles.z));
    analogWrite(LASER, !message.laserDisable * 200);

    attitude->updateHeading(&satAngVel/*, &angles, &satAttitude*/);
    // Serial.println("AngVel: " + String(satAngVel.x, 2) + " " + String(satAngVel.y, 2) + " " + String(satAngVel.z, 2));
    // Serial.println(String(satAttitude.a, 6) + "\t" + String(satAttitude.b, 6) + "\t" + String(satAttitude.c, 6) + "\t" + String(satAttitude.d));
    // Serial.println(String(angles.x, 6) + " " + String(angles.y, 6) + " " + String(angles.z, 6));

    motors->updateMotor(&message);
    radio->sendMessage(&satAngVel);

    delay(100);
}
