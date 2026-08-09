#include "HeartSensor.h"

HeartSensor::HeartSensor(int inputPin, int simButtonPin)
    : pin(inputPin), buttonPin(simButtonPin),
      noPulseResponse(false), responseStartTime(0),
      lastBeatTime(0), hrLastState(LOW), bpm(0), invalidPulseCount(0),
      bpmReadIndex(0), averageBPM(0), isBpmReady(false), contactDetected(true),
      contactLostNotified(false), lastHrReadTime(0) {
    for (int i = 0; i < NUM_BPM_READINGS; i++) {
        bpmReadings[i] = 0;
    }
}

void HeartSensor::init() {
    pinMode(buttonPin, INPUT_PULLUP);
    pinMode(pin, INPUT);
    lastBeatTime = millis();

    Serial.println("=================================================");
    Serial.println(" SmartChair Safety Control System                ");
    Serial.println("=================================================");
}

void HeartSensor::update(unsigned long currentTime) {
    // 버튼 입력에 따른 안전 대응은 리셋 전까지 유지한다.
    if (digitalRead(buttonPin) == LOW && !noPulseResponse) {
        noPulseResponse = true;
        responseStartTime = currentTime;
        averageBPM = 0;
        Serial.println("\n[상태] 테스트 버튼 입력: 무맥박 상태 대응을 시작합니다.\n");
    }

    if (!noPulseResponse) {
        if (currentTime - lastHrReadTime >= HEART_SAMPLE_INTERVAL) {
            lastHrReadTime = currentTime;
            bool hrCurrentState = (digitalRead(pin) == HIGH);

            if (hrCurrentState == HIGH && hrLastState == LOW) {
                unsigned long ibi = currentTime - lastBeatTime;
                lastBeatTime = currentTime;

                // 정상 맥박 범위(30~200 BPM)
                if (ibi > MIN_VALID_IBI && ibi < MAX_VALID_IBI) {
                    invalidPulseCount = 0;
                    bpm = static_cast<int>(MILLISECONDS_PER_MINUTE / ibi);

                    bpmReadings[bpmReadIndex] = bpm;
                    bpmReadIndex++;
                    if (bpmReadIndex >= NUM_BPM_READINGS) {
                        bpmReadIndex = 0;
                        isBpmReady = true;
                    }

                    if (isBpmReady) {
                        int highNoiseCount = 0;
                        for (int i = 0; i < NUM_BPM_READINGS; i++) {
                            if (bpmReadings[i] >= CONTACT_LOST_BPM_THRESHOLD) {
                                highNoiseCount++;
                            }
                        }

                        if (highNoiseCount >= CONTACT_LOST_SAMPLE_COUNT) {
                            contactDetected = false;
                            if (!contactLostNotified) {
                                Serial.println("[상태] 센서 접촉 해제 상태로 판정되었습니다. 제동 판단에서 제외합니다.");
                                contactLostNotified = true;
                            }
                        } else {
                            contactDetected = true;
                            if (contactLostNotified) {
                                Serial.println("[상태] 센서 접촉이 확인되었습니다. 측정을 재개합니다.");
                                contactLostNotified = false;
                            }

                            int sortedBpm[NUM_BPM_READINGS];
                            for (int i = 0; i < NUM_BPM_READINGS; i++) {
                                sortedBpm[i] = bpmReadings[i];
                            }

                            for (int i = 0; i < NUM_BPM_READINGS - 1; i++) {
                                for (int j = 0; j < NUM_BPM_READINGS - i - 1; j++) {
                                    if (sortedBpm[j] > sortedBpm[j + 1]) {
                                        int temp = sortedBpm[j];
                                        sortedBpm[j] = sortedBpm[j + 1];
                                        sortedBpm[j + 1] = temp;
                                    }
                                }
                            }

                            averageBPM = (sortedBpm[1] + sortedBpm[2] + sortedBpm[3]) /
                                         TRIMMED_AVERAGE_COUNT;
                        }
                    } else {
                        // 샘플 5개가 모이기 전에는 센서 접촉 상태로 처리한다.
                        contactDetected = true;
                        contactLostNotified = false;
                    }

                    if (isBpmReady && contactDetected &&
                        averageBPM < NO_PULSE_BPM_THRESHOLD && !noPulseResponse) {
                        noPulseResponse = true;
                        responseStartTime = currentTime;
                        averageBPM = 0;
                        Serial.println("\n[상태] 평균 심박 저하 감지: 완만한 안전 제동을 시작합니다.\n");
                    }
                } else {
                    invalidPulseCount++;
                    if (invalidPulseCount >= INVALID_PULSE_LIMIT) {
                        contactDetected = false;
                        if (!contactLostNotified) {
                            Serial.println("[상태] 반복적인 신호 이상 감지: 센서 접촉 해제 상태로 전환합니다.");
                            contactLostNotified = true;
                        }
                    }
                }
            }

            hrLastState = hrCurrentState;
        }
    } else {
        averageBPM = 0;
    }

    if (isBpmReady && contactDetected && !noPulseResponse &&
        currentTime - lastBeatTime > NO_PULSE_TIMEOUT) {
        noPulseResponse = true;
        responseStartTime = currentTime;
        averageBPM = 0;
        Serial.println("\n[상태] 2초간 무맥박 상태 감지: 완만한 안전 제동을 시작합니다.\n");
    }
}

bool HeartSensor::isNoPulseResponse() const {
    return noPulseResponse;
}

unsigned long HeartSensor::getResponseStartTime() const {
    return responseStartTime;
}

bool HeartSensor::isContactDetected() const {
    return contactDetected;
}

bool HeartSensor::hasAverageBPM() const {
    return isBpmReady;
}

int HeartSensor::getAverageBPM() const {
    return averageBPM;
}
