#include "HeartSensor.h"
#include "Config.h" // NUM_READINGS 매크로 참조를 위해 추가

HeartSensor::HeartSensor(int inputPin) : pin(inputPin), currentState(STATE_SETTLING), 
    calMax(0), calMin(1023), threshold(0), upperLimit(1023), lowerLimit(0), 
    isCalibrated(false), readIndex(0), total(0), averageHeartValue(0), 
    lastBeatTime(0), isBeat(false), lastPrintTime(0) {
    for (int i = 0; i < NUM_READINGS; i++) {
        readings[i] = 0;
    }
}

void HeartSensor::init() {
    bootTime = millis();
    Serial.println("➔ [시스템 기동] 스마트 휠체어 안전 제어 시스템을 시작합니다.");
}

void HeartSensor::update(unsigned long currentTime) {
    unsigned long elapsedTime = currentTime - bootTime;
    int rawValue = analogRead(pin);

    // 이동 평균 필터 적용 (노이즈 감쇄)
    total = total - readings[readIndex];
    readings[readIndex] = rawValue;
    total = total + readings[readIndex];
    
    readIndex = readIndex + 1;
    if (readIndex >= NUM_READINGS) readIndex = 0;
    
    averageHeartValue = total / NUM_READINGS;

    // 단계별 시스템 캘리브레이션 제어
    if (!isCalibrated) {
        if (elapsedTime < 3000) {
            currentState = STATE_SETTLING;
            executeDebugPrint(currentTime);
            return;
        } 
        else if (elapsedTime >= 3000 && elapsedTime < 8000) {
            currentState = STATE_CALIBRATING;
            if (averageHeartValue > calMax) calMax = averageHeartValue;
            if (averageHeartValue < calMin) calMin = averageHeartValue;
            executeDebugPrint(currentTime);
            return;
        } 
        else {
            if ((calMax - calMin) < 30) {
                Serial.println("⚠ 오류: 맥박 파형이 너무 약합니다. 캘리브레이션을 재시작합니다.");
                bootTime = currentTime;
                calMax = 0; calMin = 1023;  
                return;
            }
            
            threshold = calMin + ((calMax - calMin) * 0.6);
            int margin = (calMax - calMin) * 2; 
            upperLimit = calMax + margin;
            lowerLimit = calMin - margin;
            
            if (upperLimit > 1023) upperLimit = 1020;
            if (lowerLimit < 0) lowerLimit = 10;

            isCalibrated = true;
            currentState = STATE_NORMAL;
            lastBeatTime = currentTime; 
            
            Serial.println("\n✅ 학습 완료! 주행을 시작합니다.");
            Serial.print(">> 맥박 카운트 기준(Threshold): "); Serial.println(threshold);
            Serial.print(">> 🚨 심정지/이탈 하한선(Lower): "); Serial.println(lowerLimit);
            Serial.print(">> 🚨 패닉/과흥분 상한선(Upper): "); Serial.println(upperLimit);
            Serial.println("-------------------------------------\n");
        }
    }

    // 임계값 초과/미만인 경우 즉각적인 비상 상태 판단
    if (isCalibrated) {
        if (averageHeartValue > upperLimit) {
            currentState = STATE_PANIC;
        } 
        else if (averageHeartValue < lowerLimit) {
            currentState = STATE_CARDIAC_ARREST;
        }
    }

    // 정상 범위 내에서 박동 유무 판단 및 타임아웃 계산
    if (currentState != STATE_PANIC && currentState != STATE_CARDIAC_ARREST) {
        if (averageHeartValue > threshold && !isBeat) {
            if (currentTime - lastBeatTime > 300) { // 300ms 불응기 설정 (오탐 방지)
                lastBeatTime = currentTime; 
                isBeat = true;
            }
        } else if (averageHeartValue < threshold) {
            isBeat = false; 
        }

        // 마지막 심박 이후 경과 시간 기준으로 FSM 상태 전이
        unsigned long timeSinceLastBeat = currentTime - lastBeatTime;

        if (timeSinceLastBeat > ARREST_TIMEOUT) {
            currentState = STATE_CARDIAC_ARREST;
        } 
        else if (timeSinceLastBeat > DANGER_TIMEOUT) {
            currentState = STATE_DANGER_SUSPECTED;
        } 
        else {
            currentState = STATE_NORMAL;
        }
    }

    executeDebugPrint(currentTime);
}

SystemState HeartSensor::getState() const {
    return currentState;
}

int HeartSensor::getAverageValue() const {
    return averageHeartValue;
}

void HeartSensor::executeDebugPrint(unsigned long currentTime) {
    // 상태 변동 발생 시 1회만 디버그 메시지를 출력하기 위한 변수
    static SystemState lastLoggedState = (SystemState)-1; 
    
    // 초기 보정 단계 진행 상태 모니터링 출력
    if (currentState == STATE_SETTLING || currentState == STATE_CALIBRATING) {
        if (lastLoggedState != currentState) {
            lastLoggedState = currentState; 
            
            if (currentState == STATE_SETTLING) {
                Serial.println("■ [0~3초] 밴드 착용 및 센서 안정화 대기 중...");
            } 
            else if (currentState == STATE_CALIBRATING) {
                Serial.println("■ [3~8초] 파형 학습 중... (가만히 계세요)");
            }
        }
        return; 
    }
}