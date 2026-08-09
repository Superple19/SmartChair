#ifndef SERVO_BRAKE_H
#define SERVO_BRAKE_H

#include <Arduino.h>
#include <Servo.h>

class ServoBrake {
private:
    Servo rightServo;
    Servo leftServo;

public:
    void init();
    void writeAngle(int rightAngle);
    void writeMaxBrake();
};

#endif
