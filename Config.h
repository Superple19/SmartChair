#ifndef CONFIG_H
#define CONFIG_H

// 핀 설정 (Arduino Mega 2560 기준 - 고정 값)
#define HEART_PIN A0
#define LEFT_SERVO_PIN 8
#define RIGHT_SERVO_PIN 9

// 심박 센서 필터 및 타이머 설정
#define NUM_READINGS 10
#define DANGER_TIMEOUT 3000 // 추후 탑승자 실측 후 수정 필요 (위험 의심 시간 ms)
#define ARREST_TIMEOUT 5000 // 추후 탑승자 실측 후 수정 필요 (심정지 판단 시간 ms)

// 서보 제동 각도 튜닝 매직넘버 (초기 대칭 임시 설정)
#define LEFT_RELEASE_ANGLE 0   // 추후 기구부 실측 후 수정 필요 (바퀴 해제 각도)
#define RIGHT_RELEASE_ANGLE 0  // 추후 기구부 실측 후 수정 필요 (바퀴 해제 각도)
#define LEFT_ENGAGE_ANGLE 90   // 추후 기구부 실측 후 수정 필요 (바퀴 잠금 각도)
#define RIGHT_ENGAGE_ANGLE 90  // 추후 기구부 실측 후 수정 필요 (바퀴 잠금 각도)

// 라이다 확장 제어 시스템 파라미터
#define MAX_DECELERATION 25.0 // 추후 주행 실측 후 수정 필요 (실측 최대 감속도 cm/s²)
#define SAFETY_MARGIN 30.0    // 추후 주행 실측 후 수정 필요 (원거리 대응 여유 마진 cm)

#endif