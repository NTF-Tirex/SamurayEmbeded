
#include "fosfor_light_controller.h"

#if defined(DEVICE_CONFIG_FOSFOR)

void FosforLightController::begin() {
    pinMode(PWM_PIN, OUTPUT);
    digitalWrite(PWM_PIN, LOW);

    Timer2.setFrequency(10000UL);
    Timer2.enableISR();
}

uint8_t FosforLightController::readStoredBrightnessPercent() const {
    uint8_t value = EEPROM.read(LightControlConfig::EEPROMAddressLevelMode);
    if (value == LightControlConfig::EEPROMEmptyByte || value > 100) {
        return LightControlConfig::DefaultStandbyBrightnessPercent;
    }
    return value;
}

void FosforLightController::writeStoredBrightnessPercent(uint8_t value) {
    if (value > 100) {
        value = 100;
    }
    EEPROM.write(LightControlConfig::EEPROMAddressLevelMode, value);
}

void FosforLightController::setBright(uint8_t percent) {
    if (percent > 100) {
        percent = 100;
    }

    if (percent > 0) {
        percent = LightControlConfig::MinBrightnessPercent +
                  ((uint16_t)(100 - LightControlConfig::MinBrightnessPercent) * percent) / 100;
    }

    duty_ = (255 * percent) / 100;
}

void FosforLightController::applyBrightnessValue(uint8_t value) {
    if (value > 100) {
        value = 100;
    }
    writeStoredBrightnessPercent(value);
    applyLightState();
}

void FosforLightController::applyLightState() {
    uint8_t modeState = 0;
    uint8_t onOffState = 0;

    if (isExternalControl) {
        modeState = EEPROM.read(LightControlConfig::EEPROMAddressModeCurrent) ? 1 : 0;
        onOffState = EEPROM.read(LightControlConfig::EEPROMAddressOnOff) ? 1 : 0;
    } else if (DEVICE_HAS_LOCAL_LIGHT_INPUTS) {
        modeState = readPhysicalSensorState(SENSOR_MODE_INDEX);
        onOffState = readPhysicalSensorState(SENSOR_ONOFF_INDEX);
    } else {
        modeState = EEPROM.read(LightControlConfig::EEPROMAddressModeCurrent) ? 1 : 0;
        onOffState = EEPROM.read(LightControlConfig::EEPROMAddressOnOff) ? 1 : 0;
    }

    if (onOffState == 0) {
        lightEnabled_ = false;
        lightBrightness_ = false;
        duty_ = 0;
        digitalWrite(PWM_PIN, LOW);
        return;
    }

    lightEnabled_ = true;

    if (modeState == 1) {
        setBright(100);
    } else {
        setBright(readStoredBrightnessPercent());
    }
}

void FosforLightController::onTimerTick() {
    if (counterBright_ == 0) {
        lightBrightness_ = duty_ > 0;
    }

    if (counterBright_ == duty_) {
        lightBrightness_ = false;
    }

    counterBright_++;

    if (lightEnabled_ && lightBrightness_) {
        digitalWrite(PWM_PIN, HIGH);
    } else {
        digitalWrite(PWM_PIN, LOW);
    }

    counterLight_++;
    if (counterLight_ >= 10000UL) {
        counterLight_ = 0;
        tickTimer();
    }
}

#endif
