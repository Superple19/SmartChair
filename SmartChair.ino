#include "Config.h"
#include "HeartSensor.h"
#include "LidarSensor.h"
#include "BrakeController.h"
#include "ServoBrake.h"

HeartSensor heart(HEART_PIN, BUTTON_PIN);
LidarSensor leftLidar(&Serial1);
LidarSensor rightLidar(&Serial2);
BrakeController brakeController;
ServoBrake brake;

void setup() {
    Serial.begin(DEBUG_BAUD_RATE);
    heart.init();
    leftLidar.init();
    rightLidar.init();
    brake.init();
}

void loop() {
    unsigned long currentTime = millis();

    heart.update(currentTime);
    leftLidar.update(currentTime);
    rightLidar.update(currentTime);

    int leftTargetAngle = leftLidar.calculateBrakeAngle();
    int rightTargetAngle = rightLidar.calculateBrakeAngle();

    int rightFinalAngle = brakeController.update(
        currentTime,
        leftTargetAngle,
        rightTargetAngle,
        heart.isNoPulseResponse(),
        heart.getResponseStartTime());

    brake.writeAngle(rightFinalAngle);

    static unsigned long lastPrintTime = 0;
    if (currentTime - lastPrintTime >= TELEMETRY_INTERVAL) {
        lastPrintTime = currentTime;

        int leftFinalAngle = SERVO_MAX_ANGLE - rightFinalAngle;

        Serial.print("좌측 속도: ");
        Serial.print((int)leftLidar.getSpeed());
        Serial.print(" | 우측 속도: ");
        Serial.print((int)rightLidar.getSpeed());

        Serial.print(" | 평균 BPM: ");
        if (heart.isNoPulseResponse()) {
            Serial.print("0");
        } else if (!heart.isContactDetected()) {
            Serial.print("--(센서 접촉 확인 필요)");
        } else if (heart.hasAverageBPM()) {
            Serial.print(heart.getAverageBPM());
        } else {
            Serial.print("--(측정 준비 중)");
        }

        Serial.print(" | 좌측 제동각: ");
        Serial.print(leftFinalAngle);
        Serial.print(" | 우측 제동각: ");
        Serial.print(rightFinalAngle);

        if (heart.isNoPulseResponse()) {
            Serial.print("도 [무맥박 상태 대응 제동 중] 제동률: ");
            Serial.print(map(rightFinalAngle, SERVO_MAX_ANGLE, SERVO_MIN_ANGLE,
                            SERVO_MIN_ANGLE, BRAKE_PERCENT_MAX));
            Serial.println("%");
        } else if (brakeController.isHoldActive(currentTime) &&
                   brakeController.getApproachCount() == 0) {
            Serial.println("도 [자동 제동 유지 중]");
        } else if (rightFinalAngle <= BRAKE_FULL_ANGLE) {
            Serial.println("도 [즉시 제동]");
        } else if (rightFinalAngle < BRAKE_RELEASE_ANGLE) {
            Serial.println("도 [비례 제동]");
        } else {
            Serial.println("도 [제동 해제]");
        }
    }
}
