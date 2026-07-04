#include "LidarSensor.h"
#include "Config.h"

LidarSensor::LidarSensor(HardwareSerial* p) : port(p), dist(0), strength(0), 
    prevDist(0), speed(0.0), ttc(99.0), lastTime(0) {}

void LidarSensor::init() {
    port->begin(115200);
}

void LidarSensor::update(unsigned long currentTime) {
    // TF-Luna 표준 9바이트 패킷 파싱
    if (port->available() >= 9) {
        if (port->read() == 0x59) {
            if (port->peek() == 0x59) { 
                port->read(); // 두 번째 0x59 처리
                
                int d_L = port->read(); int d_H = port->read();
                int s_L = port->read(); int s_H = port->read();
                for(int i = 0; i < 3; i++) { port->read(); } // 나머지 버퍼 폐기
                
                dist = d_L + (d_H * 256); 
                strength = s_L + (s_H * 256);

                if (currentTime - lastTime >= 100) {
                    float dt = (currentTime - lastTime) / 1000.0;
                    if (dt > 0.001 && strength > 100 && dist > 0 && prevDist > 0) {
                        speed = (prevDist - dist) / dt; 
                        if (speed > 0) ttc = (float)dist / speed;
                        else ttc = 99.0;
                    } else {
                        speed = 0; ttc = 99.0;
                    }
                    prevDist = dist; 
                    lastTime = currentTime;
                }
            }
        }
    }
}

int LidarSensor::calculateBrakeAngle() const {
    if (strength <= 100 || dist <= 0 || dist > 800) return 0;

    // 비상 제동 조건 판단 (50cm 이하 인접 또는 0.4초 이내 충돌 예정인 경우)
    if (dist <= 50 || ttc <= 0.4) {
        return 90; 
    }

    // 차량 속도 및 감속 성능 상수 기준 필요 제동 거리 연산
    float requiredBrakingDistance = 0.0;
    if (speed > 0) {
        requiredBrakingDistance = (speed * speed) / (2.0 * MAX_DECELERATION);
    }
    float dangerBoundary = requiredBrakingDistance + SAFETY_MARGIN;

    // 안전 지역 진입 여부 판정 (3m 이상 유지, 충돌 위험선 외부, TTC 3초 이상 확보)
    if (dist > 300 && dist > dangerBoundary && ttc > 3.0) {
        return 0; 
    }

    // 제동 경계선 진입 시 감속 비례 제어 (각도 20도~80도 맵핑)
    if (dist <= dangerBoundary) {
        int proportionalAngle = map(dist, (long)dangerBoundary, 50, 20, 80);
        return constrain(proportionalAngle, 20, 80);
    } 
    
    return 0;
}

int LidarSensor::getDistance() const { return dist; }
float LidarSensor::getTTC() const { return ttc; }
float LidarSensor::getSpeed() const { return speed; }