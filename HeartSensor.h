#ifndef HEART_SENSOR_H
#define HEART_SENSOR_H

#include <Arduino.h>
#include "Config.h"

enum SystemState {
    STATE_SETTLING,          // 0단계: 기동 후 센서 안정화 대기
    STATE_CALIBRATING,       // 1단계: 사용자 개별 맥박 학습
    STATE_NORMAL,            // 2단계: 정상 주행 상태
    STATE_DANGER_SUSPECTED,  // 3단계: 무맥박 위험 의심 (3초 대기)
    STATE_CARDIAC_ARREST,    // 4단계: 심정지 또는 센서 이탈 확정
    STATE_PANIC              // 5단계: 패닉 및 과흥분 확정
};

class HeartSensor {
private:
    int pin;
    SystemState currentState;
    unsigned long bootTime;
    int calMax;
    int calMin;
    int threshold;
    int upperLimit;
    int lowerLimit;
    bool isCalibrated;

    int readings[NUM_READINGS];
    int readIndex;
    long total;
    int averageHeartValue;

    unsigned long lastBeatTime;
    bool isBeat;
    unsigned long lastPrintTime;

    void executeDebugPrint(unsigned long currentTime);

public:
    HeartSensor(int inputPin);
    void init();
    void update(unsigned long currentTime);
    SystemState getState() const;
    int getAverageValue() const;
};

#endif