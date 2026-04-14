
#pragma once

#include "light_controller_common.h"

#if defined(DEVICE_CONFIG_FOSFOR)
class FosforLightController : public ILightController {
public:
    void begin() override;
    void applyLightState() override;
    void applyBrightnessValue(uint8_t value) override;
    uint8_t readStoredBrightnessPercent() const override;
    void writeStoredBrightnessPercent(uint8_t value) override;
    void onTimerTick() override;

private:
    int duty_ = 0;
    bool lightEnabled_ = false;
    bool lightBrightness_ = false;
    uint8_t counterBright_ = 0;
    uint32_t counterLight_ = 0;

    void setBright(uint8_t percent);
};
#endif
