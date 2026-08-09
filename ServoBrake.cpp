#include "ServoBrake.h"
#include "Config.h"

void ServoBrake::init() {
    rightServo.attach(RIGHT_SERVO_PIN);
    leftServo.attach(LEFT_SERVO_PIN);

    // 초기 위치: 제동 해제
    writeAngle(SERVO_MAX_ANGLE);
}

void ServoBrake::writeAngle(int rightAngle) {
    rightAngle = constrain(rightAngle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);

    // 좌측은 기준 제동량, 우측은 장력 보정을 적용한다.
    int leftAngle = SERVO_MAX_ANGLE - rightAngle;
    int rightBrakeProgress = leftAngle;
    int adjustedRightAngle = SERVO_MAX_ANGLE -
        (int)(rightBrakeProgress * RIGHT_BRAKE_SCALE + SERVO_ROUNDING_OFFSET);

    rightServo.write(constrain(adjustedRightAngle, SERVO_MIN_ANGLE,
                               SERVO_MAX_ANGLE));
    leftServo.write(leftAngle);
}

void ServoBrake::writeMaxBrake() {
    writeAngle(SERVO_MIN_ANGLE);
}
