#include "BrakeController.h"
#include "Config.h"

BrakeController::BrakeController()
    : lastLogicTime(0), rawTargetAngle(BRAKE_RELEASE_ANGLE), approachCount(0),
      immediateBrakeCount(0), validStop(false), brakeHoldUntil(0),
      heldBrakeAngle(BRAKE_RELEASE_ANGLE) {}

int BrakeController::update(unsigned long currentTime,
                            int leftTargetAngle,
                            int rightTargetAngle,
                            bool noPulseResponse,
                            unsigned long responseStartTime) {
    // 100ms 주기로 양측 라이다의 접근 신호를 융합한다.
    if (currentTime - lastLogicTime >= BRAKE_CONTROL_INTERVAL) {
        lastLogicTime = currentTime;

        // 제동각은 값이 작을수록 제동 강도가 커진다.
        rawTargetAngle = min(leftTargetAngle, rightTargetAngle);

        if (rawTargetAngle < BRAKE_RELEASE_ANGLE) {
            approachCount++;

            if (rawTargetAngle == BRAKE_FULL_ANGLE) {
                immediateBrakeCount++;
            } else {
                immediateBrakeCount = 0;
            }

            if (rawTargetAngle > BRAKE_FULL_ANGLE &&
                approachCount >= LONG_BRAKE_FRAMES) {
                validStop = true;
            }
            if (rawTargetAngle == BRAKE_FULL_ANGLE &&
                immediateBrakeCount >= IMMEDIATE_BRAKE_HOLD_FRAMES) {
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

    int rightFinalAngle = (currentTime < brakeHoldUntil)
                              ? heldBrakeAngle
                              : BRAKE_RELEASE_ANGLE;

    if (approachCount >= REQUIRED_APPROACH_FRAMES) {
        rightFinalAngle = rawTargetAngle;
        heldBrakeAngle = rightFinalAngle;
    }

    if (noPulseResponse) {
        unsigned long elapsedResponseTime = currentTime - responseStartTime;
        int responseTargetAngle = BRAKE_RELEASE_ANGLE;

        if (elapsedResponseTime >= NO_PULSE_BRAKING_TIME) {
            responseTargetAngle = BRAKE_FULL_ANGLE;
        } else {
            responseTargetAngle = map(
                elapsedResponseTime, 0, NO_PULSE_BRAKING_TIME,
                BRAKE_RELEASE_ANGLE, BRAKE_FULL_ANGLE);
        }

        rightFinalAngle = min(rightFinalAngle, responseTargetAngle);
    }

    return constrain(rightFinalAngle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
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
