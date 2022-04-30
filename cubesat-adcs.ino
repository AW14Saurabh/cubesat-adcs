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
    Serial.begin(115200);
    pinMode(BEEPER, OUTPUT);
    pinMode(LASER,  OUTPUT);
    analogWrite(LASER, 200);
    radio = new Radio_Communication();
    // Serial.println("Radio On");
    motors = new Motor_Control(&satAngVel, &angles);
    // Serial.println("Motors On");
    attitude = new Attitude_Determination();
    Serial.println("Gyro On");
    tone(BEEPER, 5000, 500);
    analogWrite(LASER, 0);
    // delay(200);
}

void loop()
{
    currentMillis = millis();
    dt = currentMillis - previousMillis;

    radio->getMessage(&message);
    Serial.print("Operation: " + String(message.opMode) + " Laser: " + message.laserDisable?"Off ":"On ");
    Serial.println("Target Angle: " + String(message.targetAngles.z));
    // delay(100);
    analogWrite(LASER, !message.laserDisable * 200);

    attitude->updateHeading(&satAngVel, &satAttitude, dt);
    attitude->getAngles(&angles, &satAttitude); //To Serial

    motors->updateMotor(&message, dt);
    // Serial.println("Motors Updated");
    radio->sendMessage(&angles);

    previousMillis = currentMillis;
    delay(1000);
}
