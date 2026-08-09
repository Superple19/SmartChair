#ifndef HEART_SENSOR_H
#define HEART_SENSOR_H

#include <Arduino.h>
#include "Config.h"

class HeartSensor {
private:
    int pin;
    int buttonPin;

    bool noPulseResponse;
    unsigned long responseStartTime;

    unsigned long lastBeatTime;
    bool hrLastState;
    int bpm;
    int invalidPulseCount;

    int bpmReadings[NUM_BPM_READINGS];
    int bpmReadIndex;
    int averageBPM;
    bool isBpmReady;
    bool contactDetected;
    bool contactLostNotified;
    unsigned long lastHrReadTime;

public:
    HeartSensor(int inputPin, int simButtonPin = BUTTON_PIN);

    void init();
    void update(unsigned long currentTime);

    bool isNoPulseResponse() const;
    unsigned long getResponseStartTime() const;
    bool isContactDetected() const;
    bool hasAverageBPM() const;
    int getAverageBPM() const;
};

#endif
