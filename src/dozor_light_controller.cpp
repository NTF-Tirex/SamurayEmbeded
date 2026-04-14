#include "dozor_light_controller.h"

#if defined(DEVICE_CONFIG_DOZOR)

#include <Arduino.h>
#include <EEPROM.h>

void DozorLightController::begin() {
    pinMode(PWM_PIN, OUTPUT);
    digitalWrite(PWM_PIN, LOW);
    analogWrite(PWM_PIN, 0);
}

uint8_t DozorLightController::readStoredBrightnessPercent() const {
    uint8_t value = EEPROM.read(LightControlConfig::EEPROMAddressLevelMode);
    if (value == LightControlConfig::EEPROMEmptyByte || value == 0 || value > 100) {
        return LightControlConfig::DefaultStandbyBrightnessPercent;
    }
    return value;
}

void DozorLightController::writeStoredBrightnessPercent(uint8_t value) {
    if (value < 1) {
        value = 1;
    }
    if (value > 100) {
        value = 100;
    }
    EEPROM.update(LightControlConfig::EEPROMAddressLevelMode, value);
}

uint8_t DozorLightController::normalizeBrightnessPercent(uint8_t percent) const {
    if (percent > 100) {
        percent = 100;
    }

    if (percent > 0) {
        percent = LightControlConfig::MinBrightnessPercent +
                  ((uint16_t)(100 - LightControlConfig::MinBrightnessPercent) * percent) / 100;
    }

    return percent;
}

uint8_t DozorLightController::percentToPwm(uint8_t percent) const {
    percent = normalizeBrightnessPercent(percent);
    return static_cast<uint16_t>(percent) * 255 / 100;
}

void DozorLightController::writePwmPercent(uint8_t percent) {
    analogWrite(PWM_PIN, percentToPwm(percent));
}

void DozorLightController::applyBrightnessValue(uint8_t value) {
    writeStoredBrightnessPercent(value);
    applyLightState();
}

void DozorLightController::applyLightState() {
    const uint8_t modeState = EEPROM.read(LightControlConfig::EEPROMAddressModeCurrent) ? 1 : 0;
    const uint8_t onOffState = EEPROM.read(LightControlConfig::EEPROMAddressOnOff) ? 1 : 0;

    if (onOffState == 0) {
        analogWrite(PWM_PIN, 0);
        return;
    }

    if (modeState == 1) {
        writePwmPercent(100);
    } else {
        writePwmPercent(readStoredBrightnessPercent());
    }
}

void DozorLightController::onTimerTick() {
    // Для Dozor используем только аппаратный PWM с ноги D10.
}

#endif
