#include "ServoBrake.h"
#include "Config.h"

void ServoBrake::init() {
    rightServo.attach(RIGHT_SERVO_PIN);
    leftServo.attach(LEFT_SERVO_PIN);

    // 초기 위치: 제동 해제
    writeAngle(180);
}

void ServoBrake::writeAngle(int rightAngle) {
    rightAngle = constrain(rightAngle, 0, 180);

    // 좌측은 기준 제동량, 우측은 장력 보정을 적용한다.
    int leftAngle = 180 - rightAngle;
    int rightBrakeProgress = leftAngle;
    int adjustedRightAngle = 180 -
        (int)(rightBrakeProgress * RIGHT_BRAKE_SCALE + 0.5);

    rightServo.write(constrain(adjustedRightAngle, 0, 180));
    leftServo.write(leftAngle);
}

void ServoBrake::writeMaxBrake() {
    writeAngle(0);
}
