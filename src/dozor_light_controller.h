
#pragma once

#include "light_controller_common.h"

#if defined(DEVICE_CONFIG_DOZOR)
class DozorLightController : public ILightController {
public:
    void begin() override;
    void applyLightState() override;
    void applyBrightnessValue(uint8_t value) override;
    void onTimerTick() override;
    uint8_t readStoredBrightnessPercent() const;
    void writeStoredBrightnessPercent(uint8_t value);

private:
    uint32_t timerCounter_ = 0;

    uint8_t normalizeBrightnessPercent(uint8_t percent) const;
    uint8_t percentToPwm(uint8_t percent) const;
    void writePwmPercent(uint8_t percent);
};
#endif
