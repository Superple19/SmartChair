#include "LidarSensor.h"
#include "Config.h"

LidarSensor::LidarSensor(HardwareSerial* p)
    : port(p), dist(0), strength(0), prevDist(0), speed(0.0),
      ttc(99.0), lastTime(0) {}

void LidarSensor::init() {
    port->begin(115200);
}

void LidarSensor::update(unsigned long currentTime) {
    // TF-Luna 표준 9바이트 패킷 파싱. 현재 구현에서는 체크섬 검증을 적용하지 않는다.
    if (port->available() >= 9) {
        if (port->read() == 0x59 && port->peek() == 0x59) {
            port->read();

            int d_L = port->read();
            int d_H = port->read();
            int s_L = port->read();
            int s_H = port->read();
            for (int i = 0; i < 3; i++) {
                port->read();
            }

            int rawDist = d_L + (d_H * 256);
            int rawStrength = s_L + (s_H * 256);

            if (rawStrength >= MIN_STRENGTH_THRESHOLD &&
                rawDist > 0 && rawDist < 800) {
                dist = rawDist;
                strength = rawStrength;
            } else {
                dist = 800;
                strength = 0;
            }

            if (currentTime - lastTime >= 100) {
                float dt = (currentTime - lastTime) / 1000.0;
                if (dt > 0.001 && dist < 800 && prevDist > 0 && prevDist < 800) {
                    float calculatedSpeed = (prevDist - dist) / dt;

                    if (calculatedSpeed >= MAX_SPEED_NOISE ||
                        calculatedSpeed <= -MAX_SPEED_NOISE) {
                        speed = 0.0;
                    } else {
                        speed = calculatedSpeed;
                    }

                    if (speed > 0 && dist < 400) {
                        ttc = static_cast<float>(dist) / speed;
                    } else {
                        ttc = 99.0;
                    }
                } else {
                    speed = 0.0;
                    ttc = 99.0;
                }

                prevDist = dist;
                lastTime = currentTime;
            }
        }
    }
}

int LidarSensor::calculateBrakeAngle() const {
    // 제동각: 180=제동 해제, 0=최대 제동
    if (strength < MIN_STRENGTH_THRESHOLD || dist <= 0 || dist > 800) {
        return 180;
    }

    if (speed <= MIN_SPEED_THRESHOLD) {
        return 180;
    }

    if (dist <= 120 || ttc <= 1.0) {
        return 0;
    }

    float requiredBrakingDistance = (speed * speed) / (2.0 * MAX_DECELERATION);
    float approachBoundary = requiredBrakingDistance + SAFETY_MARGIN;

    if (dist > 400 && dist > approachBoundary && ttc > 2.5) {
        return 180;
    }

    if (dist <= approachBoundary) {
        int proportionalAngle = map(dist, (long)approachBoundary, 120, 140, 20);
        return constrain(proportionalAngle, 20, 140);
    }

    return 180;
}

int LidarSensor::getDistance() const {
    return dist;
}

float LidarSensor::getTTC() const {
    return ttc;
}

float LidarSensor::getSpeed() const {
    return speed;
}
