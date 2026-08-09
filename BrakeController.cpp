#include "BrakeController.h"
#include "Config.h"

BrakeController::BrakeController()
    : lastLogicTime(0), rawTargetAngle(180), approachCount(0),
      immediateBrakeCount(0), validStop(false), brakeHoldUntil(0),
      heldBrakeAngle(180) {}

int BrakeController::update(unsigned long currentTime,
                            int leftTargetAngle,
                            int rightTargetAngle,
                            bool noPulseResponse,
                            unsigned long responseStartTime) {
    // 100ms 주기로 양측 라이다의 접근 신호를 융합한다.
    if (currentTime - lastLogicTime >= 100) {
        lastLogicTime = currentTime;

        // 제동각은 값이 작을수록 제동 강도가 커진다.
        rawTargetAngle = min(leftTargetAngle, rightTargetAngle);

        if (rawTargetAngle < 180) {
            approachCount++;

            if (rawTargetAngle == 0) {
                immediateBrakeCount++;
            } else {
                immediateBrakeCount = 0;
            }

            if (rawTargetAngle > 0 && approachCount >= LONG_BRAKE_FRAMES) {
                validStop = true;
            }
            if (rawTargetAngle == 0 && immediateBrakeCount >= IMMEDIATE_BRAKE_HOLD_FRAMES) {
                validStop = true;
            }
        } else {
            if (validStop && approachCount >= REQUIRED_APPROACH_FRAMES) {
                brakeHoldUntil = currentTime + BRAKE_HOLD_TIME;
            }

            approachCount = 0;
            immediateBrakeCount = 0;
            validStop = false;
        }
    }

    int rightFinalAngle = (currentTime < brakeHoldUntil) ? heldBrakeAngle : 180;

    if (approachCount >= REQUIRED_APPROACH_FRAMES) {
        rightFinalAngle = rawTargetAngle;
        heldBrakeAngle = rightFinalAngle;
    }

    if (noPulseResponse) {
        unsigned long elapsedResponseTime = currentTime - responseStartTime;
        int responseTargetAngle = 180;

        if (elapsedResponseTime >= NO_PULSE_BRAKING_TIME) {
            responseTargetAngle = 0;
        } else {
            responseTargetAngle = map(
                elapsedResponseTime, 0, NO_PULSE_BRAKING_TIME, 180, 0);
        }

        rightFinalAngle = min(rightFinalAngle, responseTargetAngle);
    }

    return constrain(rightFinalAngle, 0, 180);
}

int BrakeController::getRawTargetAngle() const {
    return rawTargetAngle;
}

int BrakeController::getApproachCount() const {
    return approachCount;
}

bool BrakeController::isHoldActive(unsigned long currentTime) const {
    return currentTime < brakeHoldUntil;
}
