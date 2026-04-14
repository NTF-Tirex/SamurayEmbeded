
#pragma once

#include <Arduino.h>
#include <EEPROM.h>
#include "GyverTimers.h"
#include "device_config.h"

namespace LightControlConfig {
constexpr uint8_t EEPROMAddressModeCurrent = 1;
constexpr uint8_t EEPROMAddressOnOff = 2;
constexpr uint8_t EEPROMAddressLevelMode = 3;
constexpr uint8_t MinBrightnessPercent = 20;
constexpr uint8_t DefaultStandbyBrightnessPercent = 30;
constexpr uint8_t EEPROMEmptyByte = 0xFF;
}

extern bool isExternalControl;
extern int countWithoutExternalControl;

uint8_t readPhysicalSensorState(uint8_t index);
void tickTimer();

class ILightController {
public:
    virtual ~ILightController() {}
    virtual void begin() = 0;
    virtual void applyLightState() = 0;
    virtual void applyBrightnessValue(uint8_t value) = 0;
    virtual uint8_t readStoredBrightnessPercent() const = 0;
    virtual void writeStoredBrightnessPercent(uint8_t value) = 0;
    virtual void onTimerTick() = 0;
};
