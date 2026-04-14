#pragma once
#include <Arduino.h>
#include "device_select.h"

#if defined(DEVICE_CONFIG_FOSFOR)
#include "device_config_fosfor.h"
#elif defined(DEVICE_CONFIG_DOZOR)
#include "device_config_dozor.h"
#else
#error "Не выбрана конфигурация устройства в device_select.h"
#endif
