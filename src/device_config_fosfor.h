#pragma once
#include <Arduino.h>

#define DEVICE_CONFIG_NAME "FOSFOR"

#define DS18B20_DATA        2
#define ON_OFF_RELAY_1      3
#define ON_OFF_RELAY_2      4
#define MODE_TURN           5
#define ONOFF_TURN          6
#define PWM_PIN             7

#define ADDR_BIT_6          11
#define ADDR_BIT_5          10
#define ADDR_BIT_4          12
#define ADDR_BIT_3          9
#define ADDR_BIT_2          17
#define ADDR_BIT_1          8
#define ADDR_BIT_0          16

#define COIL_MODE_CURRENT   0
#define COIL_ONOFF          1
#define COIL_ACTUATOR_0     2
#define COIL_ACTUATOR_1     3

#define INPUT_DISCRETE_0    0
#define INPUT_DISCRETE_1    1
#define INPUT_DISCRETE_2    2
#define INPUT_DISCRETE_3    3
#define INPUT_DISCRETE_4    4
#define INPUT_DISCRETE_5    5
#define INPUT_DISCRETE_6    6
#define INPUT_DISCRETE_7    7

constexpr bool DEVICE_HAS_LOCAL_LIGHT_INPUTS = true;
constexpr uint8_t SENSOR_MODE_INDEX = 0;
constexpr uint8_t SENSOR_ONOFF_INDEX = 1;

const int coilsList[] = {
    COIL_MODE_CURRENT,
    COIL_ONOFF,
    COIL_ACTUATOR_0,
    COIL_ACTUATOR_1
};

const uint8_t sensorPins[] = { MODE_TURN, ONOFF_TURN };
const bool sensorInvert[] = { true, true };
const uint8_t actuatorPins[] = { ON_OFF_RELAY_1, ON_OFF_RELAY_2 };

const int inputDiscreteList[] = {
    INPUT_DISCRETE_0, INPUT_DISCRETE_1, INPUT_DISCRETE_2, INPUT_DISCRETE_3,
    INPUT_DISCRETE_4, INPUT_DISCRETE_5, INPUT_DISCRETE_6, INPUT_DISCRETE_7
};

constexpr size_t SENSOR_PINS_COUNT = sizeof(sensorPins) / sizeof(sensorPins[0]);
constexpr size_t ACTUATOR_PINS_COUNT = sizeof(actuatorPins) / sizeof(actuatorPins[0]);
constexpr size_t COILS_LIST_COUNT = sizeof(coilsList) / sizeof(coilsList[0]);
constexpr size_t INPUT_DISCRETE_LIST_COUNT = sizeof(inputDiscreteList) / sizeof(inputDiscreteList[0]);

struct PinInit {
    uint8_t pin;
    uint8_t mode;
    uint8_t value;
    bool setValue;
};

const PinInit pinInitList[] = {
    {ON_OFF_RELAY_1, OUTPUT, LOW, true},
    {ON_OFF_RELAY_2, OUTPUT, LOW, true},
    {MODE_TURN, INPUT_PULLUP, LOW, false},
    {ONOFF_TURN, INPUT_PULLUP, LOW, false},
    {PWM_PIN, OUTPUT, LOW, true},
    {ADDR_BIT_6, INPUT, LOW, false},
    {ADDR_BIT_5, INPUT, LOW, false},
    {ADDR_BIT_4, INPUT, LOW, false},
    {ADDR_BIT_3, INPUT, LOW, false},
    {ADDR_BIT_2, INPUT, LOW, false},
    {ADDR_BIT_1, INPUT, LOW, false},
    {ADDR_BIT_0, INPUT, LOW, false},
};

constexpr size_t PIN_INIT_COUNT = sizeof(pinInitList) / sizeof(pinInitList[0]);
