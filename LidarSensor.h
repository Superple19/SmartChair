#ifndef LIDAR_SENSOR_H
#define LIDAR_SENSOR_H

#include <Arduino.h>

class LidarSensor {
private:
    HardwareSerial* port;
    int dist;
    int strength;
    int prevDist;
    float speed;
    float ttc;
    unsigned long lastTime;

public:
    LidarSensor(HardwareSerial* p);
    void init();
    void update(unsigned long currentTime);
    int calculateBrakeAngle() const;
    int getDistance() const;
    float getTTC() const;
    float getSpeed() const;
};

#endif