
#pragma once

#include "device_config.h"

#if defined(DEVICE_CONFIG_DOZOR)
#include "dozor_light_controller.h"
using ActiveLightController = DozorLightController;
#elif defined(DEVICE_CONFIG_FOSFOR)
#include "fosfor_light_controller.h"
using ActiveLightController = FosforLightController;
#else
#error "Не выбрана конфигурация светильника"
#endif
