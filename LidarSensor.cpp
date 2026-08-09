#include "LidarSensor.h"
#include "Config.h"

LidarSensor::LidarSensor(HardwareSerial* p)
    : port(p), dist(0), strength(0), prevDist(0), speed(0.0),
      ttc(INVALID_TTC), lastTime(0) {}

void LidarSensor::init() {
    port->begin(LIDAR_BAUD_RATE);
}

void LidarSensor::update(unsigned long currentTime) {
    // TF-Luna 표준 9바이트 패킷 파싱. 현재 구현에서는 체크섬 검증을 적용하지 않는다.
    if (port->available() >= LIDAR_PACKET_SIZE) {
        if (port->read() == LIDAR_FRAME_HEADER &&
            port->peek() == LIDAR_FRAME_HEADER) {
            port->read();

            int d_L = port->read();
            int d_H = port->read();
            int s_L = port->read();
            int s_H = port->read();
            for (int i = 0; i < LIDAR_RESERVED_BYTES; i++) {
                port->read();
            }

            int rawDist = d_L + (d_H * 256);
            int rawStrength = s_L + (s_H * 256);

            if (rawStrength >= MIN_STRENGTH_THRESHOLD &&
                rawDist > LIDAR_MIN_DISTANCE && rawDist < LIDAR_MAX_DISTANCE) {
                dist = rawDist;
                strength = rawStrength;
            } else {
                dist = LIDAR_MAX_DISTANCE;
                strength = 0;
            }

            if (currentTime - lastTime >= LIDAR_SPEED_UPDATE_INTERVAL) {
                float dt = (currentTime - lastTime) / MILLISECONDS_PER_SECOND;
                if (dt > MIN_SPEED_DT && dist < LIDAR_MAX_DISTANCE &&
                    prevDist > LIDAR_MIN_DISTANCE && prevDist < LIDAR_MAX_DISTANCE) {
                    float calculatedSpeed = (prevDist - dist) / dt;

                    if (calculatedSpeed >= MAX_SPEED_NOISE ||
                        calculatedSpeed <= -MAX_SPEED_NOISE) {
                        speed = 0.0;
                    } else {
                        speed = calculatedSpeed;
                    }

                    if (speed > 0 && dist < TTC_DISTANCE_LIMIT) {
                        ttc = static_cast<float>(dist) / speed;
                    } else {
                        ttc = INVALID_TTC;
                    }
                } else {
                    speed = 0.0;
                    ttc = INVALID_TTC;
                }

                prevDist = dist;
                lastTime = currentTime;
            }
        }
    }
}

int LidarSensor::calculateBrakeAngle() const {
    // 제동각: 180=제동 해제, 0=최대 제동
    if (strength < MIN_STRENGTH_THRESHOLD || dist <= LIDAR_MIN_DISTANCE ||
        dist > LIDAR_MAX_DISTANCE) {
        return BRAKE_RELEASE_ANGLE;
    }

    if (speed <= MIN_SPEED_THRESHOLD) {
        return BRAKE_RELEASE_ANGLE;
    }

    if (dist <= IMMEDIATE_BRAKE_DISTANCE || ttc <= IMMEDIATE_BRAKE_TTC) {
        return BRAKE_FULL_ANGLE;
    }

    float requiredBrakingDistance = (speed * speed) / (2.0 * MAX_DECELERATION);
    float approachBoundary = requiredBrakingDistance + SAFETY_MARGIN;

    if (dist > TTC_DISTANCE_LIMIT && dist > approachBoundary &&
        ttc > FREE_ROLL_TTC_THRESHOLD) {
        return BRAKE_RELEASE_ANGLE;
    }

    if (dist <= approachBoundary) {
        int proportionalAngle = map(
            dist, (long)approachBoundary, IMMEDIATE_BRAKE_DISTANCE,
            PROPORTIONAL_MAX_ANGLE, PROPORTIONAL_MIN_ANGLE);
        return constrain(proportionalAngle, PROPORTIONAL_MIN_ANGLE,
                         PROPORTIONAL_MAX_ANGLE);
    }

    return BRAKE_RELEASE_ANGLE;
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
