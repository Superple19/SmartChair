#ifndef SERVO_BRAKE_H
#define SERVO_BRAKE_H

#include <Arduino.h>
#include <Servo.h>

class ServoBrake {
private:
    Servo leftServo;
    Servo rightServo;

public:
    void init();
    void writeAngle(int angle);
    void writeEmergencyStop();
};

#endif