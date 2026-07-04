#include "Config.h"
#include "HeartSensor.h"
#include "LidarSensor.h"
#include "ServoBrake.h"

// 객체 생성 (Arduino Mega 멀티 하드웨어 시리얼 포트 할당)
HeartSensor heart(HEART_PIN);
LidarSensor leftLidar(&Serial1);
LidarSensor rightLidar(&Serial2);
ServoBrake brake;

void setup() {
    Serial.begin(115200);
    heart.init();
    leftLidar.init();
    rightLidar.init();
    brake.init();
}

void loop() {
    unsigned long currentTime = millis();

    // 센서 데이터 갱신 (비동기 폴링)
    heart.update(currentTime);
    leftLidar.update(currentTime);
    rightLidar.update(currentTime);

    // 심박 상태 및 양측 라이다 기반 계산 제동각 추정
    SystemState heartState = heart.getState();
    int leftBrakeAngle = leftLidar.calculateBrakeAngle();
    int rightBrakeAngle = rightLidar.calculateBrakeAngle();

    // 두 채널 중 위험도가 높은(각도가 큰) 결과 채택
    int finalBrakeAngle = max(leftBrakeAngle, rightBrakeAngle);

    // 탑승자 건강 상태 FSM에 기반한 Fail-Safe 제동 신호 제어
    if (heartState == STATE_SETTLING || heartState == STATE_CALIBRATING) {
        // 학습 및 센서 안정화 구간에는 임의 주행 방지를 위해 긴급 제동 유지
        brake.writeEmergencyStop();
        finalBrakeAngle = 90;
    } 
    else if (heartState == STATE_CARDIAC_ARREST || heartState == STATE_PANIC) {
        // 급격한 심박 변화 또는 심정지 감지 시 즉각 완전 제동
        brake.writeEmergencyStop();
        finalBrakeAngle = 90;
    } 
    else if (heartState == STATE_DANGER_SUSPECTED) {
        // 위험 의심 상태(3초 무맥박) 진입 시 소프트 제동 적용(최소 45도 보장)
        finalBrakeAngle = max(finalBrakeAngle, 45);
        brake.writeAngle(finalBrakeAngle);
    } 
    else {
        // 정상 상태인 경우 라이다가 연산한 물리 비례 제동 각도 실시간 반영
        brake.writeAngle(finalBrakeAngle);
    }

    // 개발자 디버그용 실시간 텔레메트리 출력 (250ms 주기)
    static unsigned long lastLogTime = 0;
    if (currentTime - lastLogTime >= 250) {
        lastLogTime = currentTime;
        
        // 초기 보정 완료 후(NORMAL 이상) 주행 데이터 시리얼 출력
        if (heartState >= STATE_NORMAL) {
            
            // 고정 길이 상태 문자열 매칭 (출력 자릿수 정렬용)
            const char* fsmStr = "NORMAL ";
            if (heartState == STATE_DANGER_SUSPECTED) fsmStr = "SUSPECT";
            else if (heartState == STATE_CARDIAC_ARREST) fsmStr = "ARREST ";
            else if (heartState == STATE_PANIC)           fsmStr = "PANIC  ";

            // 시스템 FSM 및 심박 값 출력
            Serial.print("[SYS:"); Serial.print(fsmStr);
            Serial.print(" H_AVG:"); 
            int avgVal = heart.getAverageValue();
            Serial.print(avgVal);
            if (avgVal < 10) Serial.print("  ");
            else if (avgVal < 100) Serial.print(" ");
            Serial.print("] | ");
            
            // L1 라이다 실시간 데이터 출력
            Serial.print("L1[D:"); Serial.print(leftLidar.getDistance()); 
            Serial.print("cm, V:"); Serial.print((int)leftLidar.getSpeed()); Serial.print("cm/s, T:");
            if (leftLidar.getTTC() >= 99.0) {
                Serial.print("SAFE");
            } else {
                Serial.print(leftLidar.getTTC(), 1); Serial.print("s");
            }
            Serial.print("] | ");

            // L2 라이다 실시간 데이터 출력
            Serial.print("L2[D:"); Serial.print(rightLidar.getDistance()); 
            Serial.print("cm, V:"); Serial.print((int)rightLidar.getSpeed()); Serial.print("cm/s, T:");
            if (rightLidar.getTTC() >= 99.0) {
                Serial.print("SAFE");
            } else {
                Serial.print(rightLidar.getTTC(), 1); Serial.print("s");
            }
            
            // 최종 제동 서보 각도 및 인디케이터 출력
            Serial.print("] ➔ OUT:");
            if (finalBrakeAngle < 10) Serial.print(" "); 
            Serial.print(finalBrakeAngle);

            if (finalBrakeAngle >= 90) {
                Serial.println("도 🔴 [EMERGENCY]");
            } else if (finalBrakeAngle > 0) {
                Serial.println("도 🟡 [P-BRAKE]");
            } else {
                Serial.println("도 🟢 [RELEASE]");
            }
        }
    }
}