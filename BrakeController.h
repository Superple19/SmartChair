#ifndef BRAKE_CONTROLLER_H
#define BRAKE_CONTROLLER_H

#include <Arduino.h>

class BrakeController {
private:
    unsigned long lastLogicTime;
    int rawTargetAngle;
    int approachCount;
    int immediateBrakeCount;
    bool validStop;
    unsigned long brakeHoldUntil;
    int heldBrakeAngle;

public:
    BrakeController();

    int update(unsigned long currentTime,
               int leftTargetAngle,
               int rightTargetAngle,
               bool noPulseResponse,
               unsigned long responseStartTime);

    int getRawTargetAngle() const;
    int getApproachCount() const;
    bool isHoldActive(unsigned long currentTime) const;
};

#endif
