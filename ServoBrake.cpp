#include "ServoBrake.h"
#include "Config.h"

void ServoBrake::init() {
    leftServo.attach(LEFT_SERVO_PIN);
    rightServo.attach(RIGHT_SERVO_PIN);
    writeAngle(0); // 제동 해제 상태로 기동
}

void ServoBrake::writeAngle(int angle) {
    // 0~90 범위 입력을 각 서보의 튜닝 범위(RELEASE ~ ENGAGE)로 동적 맵핑 계산
    int targetLeft = map(angle, 0, 90, LEFT_RELEASE_ANGLE, LEFT_ENGAGE_ANGLE);
    int targetRight = map(angle, 0, 90, RIGHT_RELEASE_ANGLE, RIGHT_ENGAGE_ANGLE);
    
    leftServo.write(targetLeft);
    rightServo.write(targetRight);
}

void ServoBrake::writeEmergencyStop() {
    leftServo.write(LEFT_ENGAGE_ANGLE);
    rightServo.write(RIGHT_ENGAGE_ANGLE);
}