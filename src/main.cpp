#include <Arduino.h>
#include "GyverTimers.h"
#include <microDS18B20/microDS18B20.h>
#include <EEPROM.h>
#include "scenario_manager.h"
#include "device_config.h"
#include "device_light_controller.h"

//----------------------------------------------
// Регистры Modbus
//----------------------------------------------

// Регистры флагов (Coils)
#define COIL_MODE_CURRENT       0
#define COIL_ONOFF              1
#define COIL_ACTUATOR_0         2
#define COIL_ACTUATOR_1         3

// Дискретные входы (Discrete Inputs)
#define INPUT_DISCRETE_0        0
#define INPUT_DISCRETE_1        1
#define INPUT_DISCRETE_2        2
#define INPUT_DISCRETE_3        3
#define INPUT_DISCRETE_4        4
#define INPUT_DISCRETE_5        5
#define INPUT_DISCRETE_6        6
#define INPUT_DISCRETE_7        7

// Состояние устройства (Input Registers)
#define INPUT_DEVICE_TYPE_ID        0
#define INPUT_SENSOR_TEMPERATURE    1
#define INPUT_SERIAL_NUMBER         2
#define INPUT_FIRMWARE_HIGH         3
#define INPUT_FIRMWARE_LOW          4
#define INPUT_COUNT_COILS           5
#define INPUT_COUNT_DISCRETE        6
#define INPUT_COUNT_REGISTERS       7
#define INPUT_COUNT_HOLD            8
#define INPUT_COUNT_SENSOR          9
#define INPUT_COUNT_ACTUATORS       10
#define INPUT_COUNT_SCENARIOS_ACTIVE  11
#define INPUT_SCENARIOS_CRC         12

// Holding Registers
#define HOLD_LEVEL_MODE             0
#define HOLD_ACTION_CODE            1
#define HOLD_ACTION_DATA_0          2
#define HOLD_ACTION_DATA_1          3
#define HOLD_ACTION_DATA_2          4
#define HOLD_ACTION_DATA_3          5
#define HOLD_ACTION_DATA_4          6
#define HOLD_ACTION_DATA_5          7
#define HOLD_ACTION_DATA_6          8
#define HOLD_ACTION_DATA_7          9
#define HOLD_ACTION_DATA_8          10
#define HOLD_ACTION_DATA_9          11
#define HOLD_ACTION_DATA_10         12
#define HOLD_ACTION_DATA_11         13
#define HOLD_ACTION_DATA_12         14
#define HOLD_ACTION_DATA_13         15
#define HOLD_ACTION_DATA_14         16
#define HOLD_ACTION_DATA_15         17
#define HOLD_ACTION_DATA_16         18
#define HOLD_ACTION_DATA_17         19
#define HOLD_ACTION_DATA_18         20
#define HOLD_ACTION_DATA_19         21
#define HOLD_ACTION_DATA_20         22
#define HOLD_ACTION_DATA_21         23
#define HOLD_ACTION_DATA_22         24
#define HOLD_ACTION_DATA_23         25
#define HOLD_ACTION_DATA_24         26
#define HOLD_ACTION_DATA_25         27
#define HOLD_ACTION_DATA_26         28
#define HOLD_ACTION_DATA_27         29
#define HOLD_ACTION_DATA_28         30
#define HOLD_ACTION_DATA_29         31
#define HOLD_ACTION_DATA_30         32
#define HOLD_ACTION_DATA_31         33

#define ACTION_CODE_SET_MAC                 1
#define ACTION_CODE_SET_TIME                2
#define ACTION_CODE_SET_DATE                3
#define ACTION_CODE_SET_NAME                4
#define ACTION_CODE_SET_SLAVE_ID_BY_MAC     5
#define ACTION_CODE_SET_SCENARIO            6
#define ACTION_CODE_GET_SCENARIO            7
#define ACTION_CODE_GET_MAC                 8
#define ACTION_CODE_GET_TIME                9
#define ACTION_CODE_GET_DATE                10
#define ACTION_CODE_GET_NAME                11
#define ACTION_CODE_SET_SENSOR_CONFIG       12
#define ACTION_CODE_GET_SENSOR_CONFIG       13
#define ACTION_CODE_SET_ACTUATOR_CONFIG     14
#define ACTION_CODE_GET_ACTUATOR_CONFIG     15
#define ACTION_CODE_CLEAR_ALL_SCENARIOS    16
#define ACTION_CODE_SENSOR_STATE_CHANGED  17
#define ACTION_CODE_SET_BRIGHTNESS       18
#define ACTION_CODE_GET_CURRENT_BRIGHTNESS 19

// Адреса в EEPROM (только для сохраняемых данных)
#define EEPROM_ADDRESS_SLAVE_ID             0
#define EEPROM_ADDRESS_MODE_CURRENT         1
#define EEPROM_ADDRESS_ONOFF                2
#define EEPROM_ADDRESS_LEVEL_MODE           3
#define EEPROM_ADDRESS_MAC_1                4
#define EEPROM_ADDRESS_MAC_2                5
#define EEPROM_ADDRESS_MAC_3                6
#define EEPROM_ADDRESS_MAC_4                7
#define EEPROM_ADDRESS_MAC_5                8
#define EEPROM_ADDRESS_MAC_6                9
#define EEPROM_ADDRESS_NAME_0               10
#define EEPROM_ADDRESS_NAME_1               11
#define EEPROM_ADDRESS_NAME_2               12
#define EEPROM_ADDRESS_NAME_3               13
#define EEPROM_ADDRESS_NAME_4               14
#define EEPROM_ADDRESS_NAME_5               15
#define EEPROM_ADDRESS_NAME_6               16
#define EEPROM_ADDRESS_NAME_7               17
#define EEPROM_ADDRESS_NAME_8               18
#define EEPROM_ADDRESS_NAME_9               19
#define EEPROM_ADDRESS_NAME_10              20
#define EEPROM_ADDRESS_NAME_11              21
#define EEPROM_ADDRESS_NAME_12              22
#define EEPROM_ADDRESS_NAME_13              23
#define EEPROM_ADDRESS_NAME_14              24
#define EEPROM_ADDRESS_NAME_15              25
#define EEPROM_ADDRESS_NAME_16              26
#define EEPROM_ADDRESS_NAME_17              27
#define EEPROM_ADDRESS_NAME_18              28
#define EEPROM_ADDRESS_NAME_19              29
#define EEPROM_ADDRESS_NAME_20              30
#define EEPROM_ADDRESS_NAME_21              31
#define EEPROM_ADDRESS_NAME_22              32
#define EEPROM_ADDRESS_NAME_23              33
#define EEPROM_ADDRESS_NAME_24              34
#define EEPROM_ADDRESS_NAME_25              35
#define EEPROM_ADDRESS_NAME_26              36
#define EEPROM_ADDRESS_NAME_27              37
#define EEPROM_ADDRESS_NAME_28              38
#define EEPROM_ADDRESS_NAME_29              39
#define EEPROM_ADDRESS_NAME_30              40
#define EEPROM_ADDRESS_NAME_31              41
#define EEPROM_ADDRESS_SENSOR_ID_0          42
#define EEPROM_ADDRESS_SENSOR_ID_1          43
#define EEPROM_ADDRESS_SENSOR_ID_2          44
#define EEPROM_ADDRESS_SENSOR_ID_3          45
#define EEPROM_ADDRESS_SENSOR_ID_4          46
#define EEPROM_ADDRESS_SENSOR_ID_5          47
#define EEPROM_ADDRESS_SENSOR_ID_6          48
#define EEPROM_ADDRESS_SENSOR_ID_7          49
#define EEPROM_ADDRESS_ACTUATOR_ID_0        50
#define EEPROM_ADDRESS_ACTUATOR_ID_1        51
#define EEPROM_ADDRESS_ACTUATOR_ID_2        52
#define EEPROM_ADDRESS_ACTUATOR_ID_3        53
#define EEPROM_ADDRESS_ACTUATOR_ID_4        54
#define EEPROM_ADDRESS_ACTUATOR_ID_5        55
#define EEPROM_ADDRESS_ACTUATOR_ID_6        56
#define EEPROM_ADDRESS_ACTUATOR_ID_7        57

/* Константы ошибок Modbus */
#define ILLEGAL_FUNCTION    1
#define ILLEGAL_ADDRESS     2
#define ILLEGAL_DATA        3
#define DEVICE_FAILURE      4

/* Modbus функции */
#define FUNCTION_READ_COILS                 1
#define FUNCTION_READ_DISCRETE_INPUTS       2
#define FUNCTION_READ_HOLD_REGISTER         3
#define FUNCTION_READ_INPUT_REGISTERS       4
#define FUNCTION_WRITE_COIL                 5
#define FUNCTION_WRITE_REGISTER             6
#define FUNCTION_WRITE_MULTIPLE_COILS       15
#define FUNCTION_WRITE_MULTIPLE_REGISTER    16

/* Настройки */
#define LEN                         240
#define DEVICE_TYPE_ID              3
#define FIRMWARE_HIGH               1
#define FIRMWARE_LOW                0
#define SERIAL_NUMBER               1
#define SLAVE_ID_DEFAULT            128
#define MIN_BRIGHTNESS              20
#define DEFAULT_STANDBY_BRIGHTNESS_PERCENT  30
#define EEPROM_EMPTY_BYTE                0xFF
#define ACTION_DATA_SIZE            32
#define NAME_SIZE                   32

const char base[16] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};

const int inputStateList[] = {
    INPUT_DEVICE_TYPE_ID,
    INPUT_SENSOR_TEMPERATURE,
    INPUT_SERIAL_NUMBER,
    INPUT_FIRMWARE_HIGH,
    INPUT_FIRMWARE_LOW,
    INPUT_COUNT_COILS,
    INPUT_COUNT_DISCRETE,
    INPUT_COUNT_REGISTERS,
    INPUT_COUNT_HOLD,
    INPUT_COUNT_SENSOR,
    INPUT_COUNT_ACTUATORS,
    INPUT_COUNT_SCENARIOS_ACTIVE,
    INPUT_SCENARIOS_CRC
};

const int holdRegisterList[] = {
    HOLD_LEVEL_MODE,
    HOLD_ACTION_CODE,
    HOLD_ACTION_DATA_0,
    HOLD_ACTION_DATA_1,
    HOLD_ACTION_DATA_2,
    HOLD_ACTION_DATA_3,
    HOLD_ACTION_DATA_4,
    HOLD_ACTION_DATA_5,
    HOLD_ACTION_DATA_6,
    HOLD_ACTION_DATA_7,
    HOLD_ACTION_DATA_8,
    HOLD_ACTION_DATA_9,
    HOLD_ACTION_DATA_10,
    HOLD_ACTION_DATA_11,
    HOLD_ACTION_DATA_12,
    HOLD_ACTION_DATA_13,
    HOLD_ACTION_DATA_14,
    HOLD_ACTION_DATA_15,
    HOLD_ACTION_DATA_16,
    HOLD_ACTION_DATA_17,
    HOLD_ACTION_DATA_18,
    HOLD_ACTION_DATA_19,
    HOLD_ACTION_DATA_20,
    HOLD_ACTION_DATA_21,
    HOLD_ACTION_DATA_22,
    HOLD_ACTION_DATA_23,
    HOLD_ACTION_DATA_24,
    HOLD_ACTION_DATA_25,
    HOLD_ACTION_DATA_26,
    HOLD_ACTION_DATA_27,
    HOLD_ACTION_DATA_28,
    HOLD_ACTION_DATA_29,
    HOLD_ACTION_DATA_30,
    HOLD_ACTION_DATA_31
};

const int eepromAddressList[] = {
    EEPROM_ADDRESS_SLAVE_ID,
    EEPROM_ADDRESS_MODE_CURRENT,
    EEPROM_ADDRESS_ONOFF,
    EEPROM_ADDRESS_LEVEL_MODE,
    EEPROM_ADDRESS_MAC_1,
    EEPROM_ADDRESS_MAC_2,
    EEPROM_ADDRESS_MAC_3,
    EEPROM_ADDRESS_MAC_4,
    EEPROM_ADDRESS_MAC_5,
    EEPROM_ADDRESS_MAC_6,
    EEPROM_ADDRESS_NAME_0,
    EEPROM_ADDRESS_NAME_1,
    EEPROM_ADDRESS_NAME_2,
    EEPROM_ADDRESS_NAME_3,
    EEPROM_ADDRESS_NAME_4,
    EEPROM_ADDRESS_NAME_5,
    EEPROM_ADDRESS_NAME_6,
    EEPROM_ADDRESS_NAME_7,
    EEPROM_ADDRESS_NAME_8,
    EEPROM_ADDRESS_NAME_9,
    EEPROM_ADDRESS_NAME_10,
    EEPROM_ADDRESS_NAME_11,
    EEPROM_ADDRESS_NAME_12,
    EEPROM_ADDRESS_NAME_13,
    EEPROM_ADDRESS_NAME_14,
    EEPROM_ADDRESS_NAME_15,
    EEPROM_ADDRESS_NAME_16,
    EEPROM_ADDRESS_NAME_17,
    EEPROM_ADDRESS_NAME_18,
    EEPROM_ADDRESS_NAME_19,
    EEPROM_ADDRESS_NAME_20,
    EEPROM_ADDRESS_NAME_21,
    EEPROM_ADDRESS_NAME_22,
    EEPROM_ADDRESS_NAME_23,
    EEPROM_ADDRESS_NAME_24,
    EEPROM_ADDRESS_NAME_25,
    EEPROM_ADDRESS_NAME_26,
    EEPROM_ADDRESS_NAME_27,
    EEPROM_ADDRESS_NAME_28,
    EEPROM_ADDRESS_NAME_29,
    EEPROM_ADDRESS_NAME_30,
    EEPROM_ADDRESS_NAME_31,
    EEPROM_ADDRESS_SENSOR_ID_0,
    EEPROM_ADDRESS_SENSOR_ID_1,
    EEPROM_ADDRESS_SENSOR_ID_2,
    EEPROM_ADDRESS_SENSOR_ID_3,
    EEPROM_ADDRESS_SENSOR_ID_4,
    EEPROM_ADDRESS_SENSOR_ID_5,
    EEPROM_ADDRESS_SENSOR_ID_6,
    EEPROM_ADDRESS_SENSOR_ID_7,
    EEPROM_ADDRESS_ACTUATOR_ID_0,
    EEPROM_ADDRESS_ACTUATOR_ID_1,
    EEPROM_ADDRESS_ACTUATOR_ID_2,
    EEPROM_ADDRESS_ACTUATOR_ID_3,
    EEPROM_ADDRESS_ACTUATOR_ID_4,
    EEPROM_ADDRESS_ACTUATOR_ID_5,
    EEPROM_ADDRESS_ACTUATOR_ID_6,
    EEPROM_ADDRESS_ACTUATOR_ID_7
};

/* Глобальные переменные */
byte addr;
char buffSerial[LEN] {};
char RX = 0;
int count_rx = 0;
char flag_rx = 0;

int modbusAdu[LEN] = {0};
int modbusAnswer[255] = {0};

byte actionData[ACTION_DATA_SIZE] = {0};
volatile bool actionPending = false;
uint16_t pendingActionCode = 0;

byte currentDay = 0;
byte currentMonth = 0;
byte currentYear = 0;
byte currentHour = 0;
byte currentMinute = 0;
byte currentSecond = 0;

bool isExternalControl = false;
int countWithoutExternalControl = 0;
const int timeoutExternalControl = 10;

uint8_t sensorIds[sizeof(sensorPins) / sizeof(sensorPins[0])] = {0};
uint8_t actuatorIds[sizeof(actuatorPins) / sizeof(actuatorPins[0])] = {0};
uint8_t sensorStates[sizeof(sensorPins) / sizeof(sensorPins[0])] = {0};
uint8_t prevSensorStates[sizeof(sensorPins) / sizeof(sensorPins[0])] = {0};
uint8_t actuatorStates[sizeof(actuatorPins) / sizeof(actuatorPins[0])] = {0};
bool actuatorCoilForceOn[sizeof(actuatorPins) / sizeof(actuatorPins[0])] = {false};
uint8_t scenarioBlinkMode = 0;
uint8_t scenarioBlinkTime = 0;

MicroDS18B20<DS18B20_DATA> sensor;
ActiveLightController lightController;

uint8_t readStoredBrightnessPercent() {
    return lightController.readStoredBrightnessPercent();
}

void writeStoredBrightnessPercent(uint8_t value) {
    lightController.writeStoredBrightnessPercent(value);
}

void applyBrightnessValue(uint8_t value) {
    lightController.applyBrightnessValue(value);
}

uint8_t getCurrentBrightnessPercent() {
    uint8_t modeState = EEPROM.read(EEPROM_ADDRESS_MODE_CURRENT) ? 1 : 0;
    uint8_t onOffState = EEPROM.read(EEPROM_ADDRESS_ONOFF) ? 1 : 0;
    if (onOffState == 0) {
        return 0;
    }
    if (modeState == 1) {
        return 100;
    }
    return readStoredBrightnessPercent();
}

/* Прототипы функций */
void getAddress();
void setupEEPROM();
void setupPinsFromConfig();
void clearEEPROM();
void loadIoConfigFromEEPROM();
void updateConfiguredIoStates();
void applyConfiguredActuatorStates();
void processScenarioTriggers();
void processRemoteSensorStateChange(uint8_t sensorID, uint8_t sensorValue);
void executeLocalScenario(const ScenarioRecord &record);
bool hasLocalSensorID(uint8_t sensorID);
uint8_t readPhysicalSensorState(uint8_t index);
uint8_t readConfiguredSensorState(uint8_t index);
bool isSensorConfigured(uint8_t index);
bool isActuatorConfigured(uint8_t index);
void clearActionData();

uint8_t readStoredBrightnessPercent();
void writeStoredBrightnessPercent(uint8_t value);
void applyBrightnessValue(uint8_t value);

void readPacketFromPort();
void sendPacketToPort(int *data, int len);
void verifyAduData(int *data, int len);
void parseFunction(int *data);

void functionReadBits(int *data, const int *array, int arraySize);
void functionReadRegister(int *data, const int *array, int arraySize);

void functionWriteSingleCoil(int *data);
void functionWriteCoils(int *data);
void functionWriteSingleRegister(int *data);
void functionWriteMultipleRegister(int *data);

void sendError(int *data, int errorCode);

byte readCoilFromEEPROM(int index);
int readHoldRegisterFromEEPROM(int index);
byte readDiscreteInput(int index);
int readInputState(int index);

void writeAndSetCoil(int index, byte value);
void writeHoldRegister(int index, int value);
void handleActionCode(int actionCode);
bool isLeapYear(byte year);

void Clear_buff();
void tickTimer();

unsigned int getInt16FromTwoInt8(int high, int low);
void hexStringToIntArray(char str[], int *array, int *len);
int getIntFromHexChar(char sym);
void getHexFromHDec(int value, char *hexOut);
int lrc8(int *data, unsigned int len);

// =====================================================
// Основные функции
// =====================================================


void setupPinsFromConfig() {
    for (size_t i = 0; i < PIN_INIT_COUNT; i++) {
        pinMode(pinInitList[i].pin, pinInitList[i].mode);
        if (pinInitList[i].setValue) {
            digitalWrite(pinInitList[i].pin, pinInitList[i].value);
        }
    }
}

void setup() {
    Serial.begin(9600);

    setupPinsFromConfig();

    getAddress();
    setupEEPROM();
    ScenarioManager::begin();
    loadIoConfigFromEEPROM();
    updateConfiguredIoStates();
    processScenarioTriggers();
    applyConfiguredActuatorStates();
    lightController.begin();
    lightController.applyLightState();
}

void loop() {
    readPacketFromPort();

    if (actionPending) {
        actionPending = false;
        handleActionCode(pendingActionCode);
    }

    updateConfiguredIoStates();
    processScenarioTriggers();
    applyConfiguredActuatorStates();
    lightController.applyLightState();
}

// =====================================================
// Modbus обработка
// =====================================================

void readPacketFromPort() {
    if (Serial.available() > 0) {
        RX = Serial.read();
        
        if (RX == 0x3A && flag_rx == 0) {
            flag_rx = 1;
            count_rx = 0;
        } 
        else if (flag_rx == 1) {
            if (RX == 0x0A) {
                --count_rx;
                buffSerial[count_rx] = '\0';
                
                int lenght = 0;
                hexStringToIntArray(buffSerial, modbusAdu, &lenght);
                
                verifyAduData(modbusAdu, lenght);
                Clear_buff();
            } 
            else {
                if (count_rx < LEN - 1) {
                    buffSerial[count_rx++] = RX;
                }
            }
        }
    }
}

void sendPacketToPort(int *data, int len) {
    if (data[0] == 0) {
        return;
    }

    Serial.write(':');
    for (int i = 0; i < len; i++) {
        char hex[2];
        getHexFromHDec(data[i], hex);
        Serial.write(hex[0]);
        Serial.write(hex[1]);
    }
    Serial.print("\r\n");
}


void verifyAduData(int *data, int len) {
    if (len < 4) return;

    int calculatedLrc = lrc8(data, len - 1);
    if (calculatedLrc != data[len - 1]) {
        sendError(data, ILLEGAL_FUNCTION);
        return; // неверная контрольная сумма
    }

    if (data[0] != addr && data[0] != 0) {
        return; // не нам
    }

    parseFunction(data);
}

void parseFunction(int *data) {
    switch (data[1]) {
        case FUNCTION_READ_COILS:
            functionReadBits(data, coilsList, sizeof(coilsList)/sizeof(coilsList[0]));
            break;
            
        case FUNCTION_READ_DISCRETE_INPUTS:
            functionReadBits(data, inputDiscreteList, sizeof(inputDiscreteList)/sizeof(inputDiscreteList[0]));
            break;
            
        case FUNCTION_READ_INPUT_REGISTERS:
            functionReadRegister(data, inputStateList, sizeof(inputStateList)/sizeof(inputStateList[0]));
            break;
            
        case FUNCTION_READ_HOLD_REGISTER:
            functionReadRegister(data, holdRegisterList, sizeof(holdRegisterList)/sizeof(holdRegisterList[0]));
            break;
            
        case FUNCTION_WRITE_COIL:
            functionWriteSingleCoil(data);
            break;
            
        case FUNCTION_WRITE_REGISTER:
            functionWriteSingleRegister(data);
            break;
            
        case FUNCTION_WRITE_MULTIPLE_COILS:
            functionWriteCoils(data);
            break;
            
        case FUNCTION_WRITE_MULTIPLE_REGISTER:
            functionWriteMultipleRegister(data);
            break;
            
        default:
            sendError(data, ILLEGAL_FUNCTION);
            break;
    }
}

// =====================================================
// Реализация функций Modbus
// =====================================================

void functionReadBits(int *data, const int *array, int arraySize) {
    unsigned int address = getInt16FromTwoInt8(data[2], data[3]);
    unsigned int quantity = getInt16FromTwoInt8(data[4], data[5]);

    if (quantity == 0 || quantity > 2000) {
        sendError(data, ILLEGAL_DATA);
        return;
    }

    int startIndex = -1;
    for (int i = 0; i < arraySize; i++) {
        if (array[i] == (int)address) {
            if (i + (int)quantity <= arraySize) {
                startIndex = i;
                break;
            }
        }
    }

    if (startIndex == -1) {
        sendError(data, ILLEGAL_ADDRESS);
        return;
    }

    int answer[255];
    int j = 0;
    answer[j++] = data[0];
    answer[j++] = data[1];
    answer[j++] = (quantity + 7) / 8;  // byte count

    byte currentByte = 0;
    int bitCount = 0;

    for (int i = startIndex; i < startIndex + (int)quantity; i++) {
        int value = 0;
        if (array == coilsList) {
            value = readCoilFromEEPROM(i);
        } else if (array == inputDiscreteList) {
            value = readDiscreteInput(i);
        }

        if (value) {
            currentByte |= (1 << bitCount);
        }

        bitCount++;
        if (bitCount == 8 || i == startIndex + (int)quantity - 1) {
            answer[j++] = currentByte;
            currentByte = 0;
            bitCount = 0;
        }
    }

    int lrc = lrc8(answer, j);
    answer[j++] = lrc;

    sendPacketToPort(answer, j);
}

void functionReadRegister(int *data, const int *array, int arraySize) {
    unsigned int address = getInt16FromTwoInt8(data[2], data[3]);
    unsigned int quantity = getInt16FromTwoInt8(data[4], data[5]);

    if (quantity == 0 || quantity > 125) {
        sendError(data, ILLEGAL_DATA);
        return;
    }

    int startIndex = -1;
    for (int i = 0; i < arraySize; i++) {
        if (array[i] == (int)address) {
            if (i + (int)quantity <= arraySize) {
                startIndex = i;
                break;
            }
        }
    }

    if (startIndex == -1) {
        sendError(data, ILLEGAL_ADDRESS);
        return;
    }

    int answer[255];
    int j = 0;
    answer[j++] = data[0];
    answer[j++] = data[1];
    answer[j++] = quantity * 2;   // byte count

    for (int i = startIndex; i < startIndex + (int)quantity; i++) {
        int value = 0;
        if (array == inputStateList) {
            value = readInputState(i);
        } else if (array == holdRegisterList) {
            value = readHoldRegisterFromEEPROM(i);
        }

        answer[j++] = value >> 8;
        answer[j++] = value & 0xFF;
    }

    int lrc = lrc8(answer, j);
    answer[j++] = lrc;

    sendPacketToPort(answer, j);
}

void functionWriteSingleCoil(int *data) {
    unsigned int address = getInt16FromTwoInt8(data[2], data[3]);
    unsigned int value = getInt16FromTwoInt8(data[4], data[5]);

    if (value != 0x0000 && value != 0xFF00) {
        sendError(data, ILLEGAL_DATA);
        return;
    }

    int coilIndex = -1;
    for (int i = 0; i < (int)(sizeof(coilsList)/sizeof(coilsList[0])); i++) {
        if (coilsList[i] == (int)address) {
            coilIndex = i;
            break;
        }
    }

    if (coilIndex == -1) {
        sendError(data, ILLEGAL_ADDRESS);
        return;
    }

    byte coilValue = (value == 0xFF00) ? 1 : 0;
    writeAndSetCoil(coilIndex, coilValue);

    // Эхо ответа
    int answer[7];
    for (int i = 0; i < 6; i++) {
        answer[i] = data[i];
    }
    int lrc = lrc8(answer, 6);
    answer[6] = lrc;

    sendPacketToPort(answer, 7);
}

void functionWriteCoils(int *data) {
    unsigned int address = getInt16FromTwoInt8(data[2], data[3]);
    unsigned int quantity = getInt16FromTwoInt8(data[4], data[5]);
    byte byteCount = data[6];

    if (quantity == 0 || quantity > 1968) {
        sendError(data, ILLEGAL_DATA);
        return;
    }

    int startIndex = -1;
    for (int i = 0; i < (int)(sizeof(coilsList)/sizeof(coilsList[0])); i++) {
        if (coilsList[i] == (int)address) {
            if (i + quantity <= (int)(sizeof(coilsList)/sizeof(coilsList[0]))) {
                startIndex = i;
                break;
            }
        }
    }

    if (startIndex == -1) {
        sendError(data, ILLEGAL_ADDRESS);
        return;
    }

    int bitIndex = 0;
    for (int byteIdx = 0; byteIdx < byteCount; byteIdx++) {
        byte currentByte = data[7 + byteIdx];

        for (int bit = 0; bit < 8 && bitIndex < (int)quantity; bit++, bitIndex++) {
            byte value = (currentByte >> bit) & 1;
            writeAndSetCoil(startIndex + bitIndex, value);
        }
    }

    // Ответ
    int answer[7];
    for (int i = 0; i < 6; i++) {
        answer[i] = data[i];
    }
    int lrc = lrc8(answer, 6);
    answer[6] = lrc;

    sendPacketToPort(answer, 7);
}

void functionWriteSingleRegister(int *data) {
    unsigned int address = getInt16FromTwoInt8(data[2], data[3]);
    int value = getInt16FromTwoInt8(data[4], data[5]);

    int regIndex = -1;
    for (int i = 0; i < (int)(sizeof(holdRegisterList)/sizeof(holdRegisterList[0])); i++) {
        if (holdRegisterList[i] == (int)address) {
            regIndex = i;
            break;
        }
    }

    if (regIndex == -1) {
        sendError(data, ILLEGAL_ADDRESS);
        return;
    }

    writeHoldRegister(regIndex, value);

    // Эхо
    int answer[7];
    for (int i = 0; i < 6; i++) {
        answer[i] = data[i];
    }
    int lrc = lrc8(answer, 6);
    answer[6] = lrc;

    sendPacketToPort(answer, 7);
}

void functionWriteMultipleRegister(int *data) {
    unsigned int address = getInt16FromTwoInt8(data[2], data[3]);
    unsigned int quantity = getInt16FromTwoInt8(data[4], data[5]);
    byte byteCount = data[6];

    if (quantity == 0 || quantity > 123) {
        sendError(data, ILLEGAL_DATA);
        return;
    }
    int startIndex = -1;
    for (int i = 0; i < (int)(sizeof(holdRegisterList)/sizeof(holdRegisterList[0])); i++) {
        if (holdRegisterList[i] == (int)address) {
            if (i + quantity <= (int)(sizeof(holdRegisterList)/sizeof(holdRegisterList[0]))) {
                startIndex = i;
                break;
            }
        }
    }

    if (startIndex == -1) {
        sendError(data, ILLEGAL_ADDRESS);
        return;
    }
    int regOffset = 0;
    for (int i = 0; i < byteCount; i += 2) {
        int value = getInt16FromTwoInt8(data[7 + i], data[8 + i]);
        writeHoldRegister(startIndex + regOffset, value);
        regOffset++;
    }

    // Ответ
    int answer[7];
    for (int i = 0; i < 6; i++) {
        answer[i] = data[i];
    }
    int lrc = lrc8(answer, 6);
    answer[6] = lrc;
    sendPacketToPort(answer, 7);
}

void sendError(int *data, int errorCode) {
    int answer[4];
    answer[0] = data[0];
    answer[1] = data[1] | 0x80;
    answer[2] = errorCode;

    int lrc = lrc8(answer, 3);
    answer[3] = lrc;

    sendPacketToPort(answer, 4);
}

// =====================================================
// EEPROM и настройки
// =====================================================

void getAddress() {
    byte currentAddress = (digitalRead(ADDR_BIT_6) << 6) |
                          (digitalRead(ADDR_BIT_5) << 5) |
                          (digitalRead(ADDR_BIT_4) << 4) |
                          (digitalRead(ADDR_BIT_3) << 3) |
                          (digitalRead(ADDR_BIT_2) << 2) |
                          (digitalRead(ADDR_BIT_1) << 1) |
                          (digitalRead(ADDR_BIT_0) << 0);
    int savedId = EEPROM.read(EEPROM_ADDRESS_SLAVE_ID);
    if (savedId == 0) {
        addr = currentAddress;
    }
    else {
        addr = savedId;
    }
}

void setupEEPROM() {
    int coilLen = sizeof(coilsList) / sizeof(coilsList[0]);
    for (int i = 0; i < coilLen; i++) {
        byte value = readCoilFromEEPROM(i);
        writeAndSetCoil(i, value);
    }

    if (EEPROM.read(EEPROM_ADDRESS_LEVEL_MODE) == EEPROM_EMPTY_BYTE || EEPROM.read(EEPROM_ADDRESS_LEVEL_MODE) > 100) {
        writeStoredBrightnessPercent(DEFAULT_STANDBY_BRIGHTNESS_PERCENT);
    }

    for (uint8_t i = 0; i < (sizeof(actuatorPins) / sizeof(actuatorPins[0])); i++) {
        actuatorCoilForceOn[i] = false;
    }

    for (int i = 0; i < ACTION_DATA_SIZE; i++) {
        actionData[i] = 0;
    }
}

void clearEEPROM() {
    int len = sizeof(eepromAddressList) / sizeof(eepromAddressList[0]);
    for (int i = 0; i < len; i++) {
        EEPROM.write(eepromAddressList[i], 0);
    }
}

void clearActionData() {
    for (int i = 0; i < ACTION_DATA_SIZE; i++) {
        actionData[i] = 0;
    }
}

void loadIoConfigFromEEPROM() {
    const uint8_t sensorCount = sizeof(sensorPins) / sizeof(sensorPins[0]);
    const uint8_t actuatorCount = sizeof(actuatorPins) / sizeof(actuatorPins[0]);

    for (uint8_t i = 0; i < sensorCount; i++) {
        sensorIds[i] = EEPROM.read(EEPROM_ADDRESS_SENSOR_ID_0 + i);
    }
    for (uint8_t i = 0; i < actuatorCount; i++) {
        actuatorIds[i] = EEPROM.read(EEPROM_ADDRESS_ACTUATOR_ID_0 + i);
        actuatorStates[i] = 0;
    }
}

bool isSensorConfigured(uint8_t index) {
    return index < (sizeof(sensorPins) / sizeof(sensorPins[0])) && sensorIds[index] != 0;
}

bool isActuatorConfigured(uint8_t index) {
    return index < (sizeof(actuatorPins) / sizeof(actuatorPins[0])) && actuatorIds[index] != 0;
}

uint8_t readPhysicalSensorState(uint8_t index) {
    if (index >= SENSOR_PINS_COUNT) {
        return 0;
    }

    uint8_t value = digitalRead(sensorPins[index]) ? 1 : 0;
    if (sensorInvert[index]) {
        value = value ? 0 : 1;
    }
    return value;
}

uint8_t readConfiguredSensorState(uint8_t index) {
    return readPhysicalSensorState(index);
}

void updateConfiguredIoStates() {
    const uint8_t sensorCount = sizeof(sensorPins) / sizeof(sensorPins[0]);
    const uint8_t actuatorCount = sizeof(actuatorPins) / sizeof(actuatorPins[0]);

    for (uint8_t i = 0; i < sensorCount; i++) {
        prevSensorStates[i] = sensorStates[i];
        sensorStates[i] = isSensorConfigured(i) ? readConfiguredSensorState(i) : 0;
    }

    for (uint8_t i = 0; i < actuatorCount; i++) {
        if (!isActuatorConfigured(i) && !actuatorCoilForceOn[i]) {
            actuatorStates[i] = 0;
        }
    }
}

void applyConfiguredActuatorStates() {
    const uint8_t actuatorCount = sizeof(actuatorPins) / sizeof(actuatorPins[0]);
    for (uint8_t i = 0; i < actuatorCount; i++) {
        if (actuatorCoilForceOn[i]) {
            digitalWrite(actuatorPins[i], HIGH);
        } else if (isActuatorConfigured(i)) {
            digitalWrite(actuatorPins[i], actuatorStates[i] ? HIGH : LOW);
        } else {
            digitalWrite(actuatorPins[i], LOW);
        }
    }
}

bool hasLocalSensorID(uint8_t sensorID) {
    if (sensorID == 0) {
        return false;
    }

    const uint8_t sensorCount = sizeof(sensorPins) / sizeof(sensorPins[0]);
    for (uint8_t i = 0; i < sensorCount; i++) {
        if (isSensorConfigured(i) && sensorIds[i] == sensorID) {
            return true;
        }
    }

    return false;
}

void executeLocalScenario(const ScenarioRecord &record) {
    uint8_t reactionType = ScenarioManager::getReactionType(record.reactionByte);
    uint8_t actuatorIndex = ScenarioManager::getActuatorIndex(record.reactionByte);

    switch (reactionType) {
        case SCENARIO_REACTION_OUT:
            if (actuatorIndex < (sizeof(actuatorPins) / sizeof(actuatorPins[0])) && isActuatorConfigured(actuatorIndex)) {
                actuatorStates[actuatorIndex] = (record.reactionValue != 0) ? 1 : 0;
                if (!actuatorCoilForceOn[actuatorIndex]) {
                    digitalWrite(actuatorPins[actuatorIndex], actuatorStates[actuatorIndex] ? HIGH : LOW);
                }
            }
            break;

        case SCENARIO_REACTION_LUM:
            if (record.reactionValue == 0) {
                EEPROM.write(EEPROM_ADDRESS_ONOFF, 0);
                lightController.applyLightState();
            } else {
                EEPROM.write(EEPROM_ADDRESS_MODE_CURRENT, 0);
                EEPROM.write(EEPROM_ADDRESS_ONOFF, 1);
                applyBrightnessValue(record.reactionValue);
            }
            break;

        case SCENARIO_REACTION_MODE:
            scenarioBlinkMode = record.reactionValue;
            break;

        case SCENARIO_REACTION_TIME:
            scenarioBlinkTime = record.reactionValue;
            break;

        default:
            break;
    }
}

void processScenarioTriggers() {
    const uint8_t sensorCount = sizeof(sensorPins) / sizeof(sensorPins[0]);
    uint8_t scenarioCount = ScenarioManager::getScenarioCount();

    for (uint8_t sensorIndex = 0; sensorIndex < sensorCount; sensorIndex++) {
        if (!isSensorConfigured(sensorIndex)) {
            continue;
        }

        uint8_t prevValue = prevSensorStates[sensorIndex];
        uint8_t currValue = sensorStates[sensorIndex];
        if (currValue == prevValue) {
            continue;
        }

        uint8_t sensorID = sensorIds[sensorIndex];

        for (uint8_t scenarioIndex = 0; scenarioIndex < scenarioCount; scenarioIndex++) {
            ScenarioRecord record;
            if (!ScenarioManager::readScenario(scenarioIndex, record)) {
                continue;
            }
            if (!ScenarioManager::isSlotActive(record)) {
                continue;
            }
            if (!ScenarioManager::isRecordStructValid(record)) {
                continue;
            }
            if (!ScenarioManager::isRecordCrcValid(record)) {
                continue;
            }
            if (record.sensorID != sensorID) {
                continue;
            }

            bool triggerMatched = false;
            if (record.sensorValue == 1) {
                triggerMatched = (prevValue == 0 && currValue == 1);
            } else if (record.sensorValue == 2) {
                triggerMatched = (prevValue == 1 && currValue == 0);
            }
            if (!triggerMatched) {
                continue;
            }
            if (!ScenarioManager::hasTargetAddress(record, addr)) {
                continue;
            }

            executeLocalScenario(record);
        }
    }
}

void processRemoteSensorStateChange(uint8_t sensorID, uint8_t sensorValue) {
    if (sensorID == 0) {
        return;
    }

    if (hasLocalSensorID(sensorID)) {
        return;
    }

    uint8_t scenarioCount = ScenarioManager::getScenarioCount();
    for (uint8_t scenarioIndex = 0; scenarioIndex < scenarioCount; scenarioIndex++) {
        ScenarioRecord record;
        if (!ScenarioManager::readScenario(scenarioIndex, record)) {
            continue;
        }
        if (!ScenarioManager::isSlotActive(record)) {
            continue;
        }
        if (!ScenarioManager::isRecordStructValid(record)) {
            continue;
        }
        if (!ScenarioManager::isRecordCrcValid(record)) {
            continue;
        }
        if (record.sensorID != sensorID) {
            continue;
        }

        bool triggerMatched = false;
        if (record.sensorValue == 1) {
            triggerMatched = (sensorValue == 1);
        } else if (record.sensorValue == 2) {
            triggerMatched = (sensorValue == 0);
        }
        if (!triggerMatched) {
            continue;
        }
        if (!ScenarioManager::hasTargetAddress(record, addr)) {
            continue;
        }

        executeLocalScenario(record);
    }
}

// =====================================================
// Чтение значений
// =====================================================

byte readCoilFromEEPROM(int index) {
    switch (coilsList[index]) {
        case COIL_MODE_CURRENT:     return EEPROM.read(EEPROM_ADDRESS_MODE_CURRENT);
        case COIL_ONOFF:            return EEPROM.read(EEPROM_ADDRESS_ONOFF);
        case COIL_ACTUATOR_0:       return actuatorCoilForceOn[0] ? 1 : 0;
        case COIL_ACTUATOR_1:       return (sizeof(actuatorPins) / sizeof(actuatorPins[0]) > 1 && actuatorCoilForceOn[1]) ? 1 : 0;
        default:                    return 0;
    }
}

int readHoldRegisterFromEEPROM(int index) {
    switch (holdRegisterList[index]) {
        case HOLD_LEVEL_MODE:       return readStoredBrightnessPercent();
        case HOLD_ACTION_CODE:      return 0; // Код действия не хранится
        case HOLD_ACTION_DATA_0:    return actionData[0];
        case HOLD_ACTION_DATA_1:    return actionData[1];
        case HOLD_ACTION_DATA_2:    return actionData[2];
        case HOLD_ACTION_DATA_3:    return actionData[3];
        case HOLD_ACTION_DATA_4:    return actionData[4];
        case HOLD_ACTION_DATA_5:    return actionData[5];
        case HOLD_ACTION_DATA_6:    return actionData[6];
        case HOLD_ACTION_DATA_7:    return actionData[7];
        case HOLD_ACTION_DATA_8:    return actionData[8];
        case HOLD_ACTION_DATA_9:    return actionData[9];
        case HOLD_ACTION_DATA_10:   return actionData[10];
        case HOLD_ACTION_DATA_11:   return actionData[11];
        case HOLD_ACTION_DATA_12:   return actionData[12];
        case HOLD_ACTION_DATA_13:   return actionData[13];
        case HOLD_ACTION_DATA_14:   return actionData[14];
        case HOLD_ACTION_DATA_15:   return actionData[15];
        case HOLD_ACTION_DATA_16:   return actionData[16];
        case HOLD_ACTION_DATA_17:   return actionData[17];
        case HOLD_ACTION_DATA_18:   return actionData[18];
        case HOLD_ACTION_DATA_19:   return actionData[19];
        case HOLD_ACTION_DATA_20:   return actionData[20];
        case HOLD_ACTION_DATA_21:   return actionData[21];
        case HOLD_ACTION_DATA_22:   return actionData[22];
        case HOLD_ACTION_DATA_23:   return actionData[23];
        case HOLD_ACTION_DATA_24:   return actionData[24];
        case HOLD_ACTION_DATA_25:   return actionData[25];
        case HOLD_ACTION_DATA_26:   return actionData[26];
        case HOLD_ACTION_DATA_27:   return actionData[27];
        case HOLD_ACTION_DATA_28:   return actionData[28];
        case HOLD_ACTION_DATA_29:   return actionData[29];
        case HOLD_ACTION_DATA_30:   return actionData[30];
        case HOLD_ACTION_DATA_31:   return actionData[31];
        default:                    return 0;
    }
}

byte readDiscreteInput(int index) {
    const uint8_t sensorCount = sizeof(sensorPins) / sizeof(sensorPins[0]);
    const uint8_t actuatorCount = sizeof(actuatorPins) / sizeof(actuatorPins[0]);

    const uint8_t connectedSensorsStart = 0;
    const uint8_t connectedActuatorsStart = connectedSensorsStart + sensorCount;
    const uint8_t sensorStatesStart = connectedActuatorsStart + actuatorCount;
    const uint8_t actuatorStatesStart = sensorStatesStart + sensorCount;
    const uint8_t totalLogicalDiscreteCount = actuatorStatesStart + actuatorCount;

    if (index < 0 || index >= totalLogicalDiscreteCount) {
        return 0;
    }

    // 0 .. sensorCount-1
    // Признаки: датчик подключен / не подключен
    if (index < connectedActuatorsStart) {
        uint8_t sensorIndex = index - connectedSensorsStart;
        return isSensorConfigured(sensorIndex) ? 1 : 0;
    }

    // sensorCount .. sensorCount+actuatorCount-1
    // Признаки: исполнительное устройство подключено / не подключено
    if (index < sensorStatesStart) {
        uint8_t actuatorIndex = index - connectedActuatorsStart;
        return isActuatorConfigured(actuatorIndex) ? 1 : 0;
    }

    // Далее sensorCount значений
    // Состояния датчиков, но если датчик не подключен -> 0
    if (index < actuatorStatesStart) {
        uint8_t sensorIndex = index - sensorStatesStart;
        return isSensorConfigured(sensorIndex) ? sensorStates[sensorIndex] : 0;
    }

    // Далее actuatorCount значений
    // Состояния исполнительных устройств, но если устройство не подключено -> 0
    uint8_t actuatorIndex = index - actuatorStatesStart;
    if (actuatorCoilForceOn[actuatorIndex]) {
        return 1;
    }
    return isActuatorConfigured(actuatorIndex) ? actuatorStates[actuatorIndex] : 0;
}

int readInputState(int index) {
    switch (inputStateList[index]) {
        case INPUT_DEVICE_TYPE_ID:
            return DEVICE_TYPE_ID;

        case INPUT_SENSOR_TEMPERATURE:
            sensor.requestTemp();
            return (int)sensor.getTemp();

        case INPUT_SERIAL_NUMBER:
            return SERIAL_NUMBER;

        case INPUT_FIRMWARE_HIGH:
            return FIRMWARE_HIGH;

        case INPUT_FIRMWARE_LOW:
            return FIRMWARE_LOW;

        case INPUT_COUNT_COILS:
            return sizeof(coilsList) / sizeof(coilsList[0]);

        case INPUT_COUNT_DISCRETE:
            return sizeof(inputDiscreteList) / sizeof(inputDiscreteList[0]);

        case INPUT_COUNT_REGISTERS:
            return sizeof(inputStateList) / sizeof(inputStateList[0]);

        case INPUT_COUNT_HOLD:
            return sizeof(holdRegisterList) / sizeof(holdRegisterList[0]);

        case INPUT_COUNT_SENSOR:
            return sizeof(sensorPins) / sizeof(sensorPins[0]);

        case INPUT_COUNT_ACTUATORS:
            return sizeof(actuatorPins) / sizeof(actuatorPins[0]);

        case INPUT_COUNT_SCENARIOS_ACTIVE:
            {
                uint8_t totalCount = ScenarioManager::getScenarioCount();
                uint8_t activeCount = 0;
                for (uint8_t i = 0; i < totalCount; i++) {
                    ScenarioRecord record;
                    if (!ScenarioManager::readScenario(i, record)) {
                        continue;
                    }
                    if (!ScenarioManager::isSlotActive(record)) {
                        continue;
                    }
                    if (!ScenarioManager::isRecordStructValid(record)) {
                        continue;
                    }
                    if (!ScenarioManager::isRecordCrcValid(record)) {
                        continue;
                    }
                    activeCount++;
                }
                return activeCount;
            }

        case INPUT_SCENARIOS_CRC:
            return ScenarioManager::calcStoredScenariosCrc();

        default:
            return 0;
    }
}

// =====================================================
// Запись значений
// =====================================================

void writeAndSetCoil(int index, byte value) {
    switch (coilsList[index]) {
        case COIL_MODE_CURRENT:
            EEPROM.write(EEPROM_ADDRESS_MODE_CURRENT, value);
            isExternalControl = true;
            countWithoutExternalControl = 0;
            lightController.applyLightState();
            break;

        case COIL_ONOFF:
            EEPROM.write(EEPROM_ADDRESS_ONOFF, value);
            isExternalControl = true;
            countWithoutExternalControl = 0;
            lightController.applyLightState();
            break;

        case COIL_ACTUATOR_0:
            actuatorCoilForceOn[0] = (value != 0);
            actuatorStates[0] = (value != 0) ? 1 : 0;
            digitalWrite(actuatorPins[0], actuatorStates[0] ? HIGH : LOW);
            break;

        case COIL_ACTUATOR_1:
            if ((sizeof(actuatorPins) / sizeof(actuatorPins[0])) > 1) {
                actuatorCoilForceOn[1] = (value != 0);
                actuatorStates[1] = (value != 0) ? 1 : 0;
                digitalWrite(actuatorPins[1], actuatorStates[1] ? HIGH : LOW);
            }
            break;
    }
}

void writeHoldRegister(int index, int value) {
    switch (holdRegisterList[index]) {
        case HOLD_LEVEL_MODE:
            if (value < 0) value = 0;
            if (value > 100) value = 100;
            applyBrightnessValue((byte)value);
            break;
        case HOLD_ACTION_CODE:
            pendingActionCode = (uint16_t)value;
            actionPending = true;
            break;
        case HOLD_ACTION_DATA_0:
            actionData[0] = (uint8_t)(value & 0xFF);
            break;
        case HOLD_ACTION_DATA_1:
            actionData[1] = (uint8_t)(value & 0xFF);
            break;
        case HOLD_ACTION_DATA_2:
            actionData[2] = (uint8_t)(value & 0xFF);
            break;
        case HOLD_ACTION_DATA_3:
            actionData[3] = (uint8_t)(value & 0xFF);
            break;
        case HOLD_ACTION_DATA_4:
            actionData[4] = (uint8_t)(value & 0xFF);
            break;
        case HOLD_ACTION_DATA_5:
            actionData[5] = (uint8_t)(value & 0xFF);
            break;
        case HOLD_ACTION_DATA_6:
            actionData[6] = (uint8_t)(value & 0xFF);
            break;
        case HOLD_ACTION_DATA_7:
            actionData[7] = (uint8_t)(value & 0xFF);
            break;
        case HOLD_ACTION_DATA_8:
            actionData[8] = (uint8_t)(value & 0xFF);
            break;
        case HOLD_ACTION_DATA_9:
            actionData[9] = (uint8_t)(value & 0xFF);
            break;
        case HOLD_ACTION_DATA_10:
            actionData[10] = (uint8_t)(value & 0xFF);
            break;
        case HOLD_ACTION_DATA_11:
            actionData[11] = (uint8_t)(value & 0xFF);
            break;
        case HOLD_ACTION_DATA_12:
            actionData[12] = (uint8_t)(value & 0xFF);
            break;
        case HOLD_ACTION_DATA_13:
            actionData[13] = (uint8_t)(value & 0xFF);
            break;
        case HOLD_ACTION_DATA_14:
            actionData[14] = (uint8_t)(value & 0xFF);
            break;
        case HOLD_ACTION_DATA_15:
            actionData[15] = (uint8_t)(value & 0xFF);
            break;
        case HOLD_ACTION_DATA_16:
            actionData[16] = (uint8_t)(value & 0xFF);
            break;
        case HOLD_ACTION_DATA_17:
            actionData[17] = (uint8_t)(value & 0xFF);
            break;
        case HOLD_ACTION_DATA_18:
            actionData[18] = (uint8_t)(value & 0xFF);
            break;
        case HOLD_ACTION_DATA_19:
            actionData[19] = (uint8_t)(value & 0xFF);
            break;
        case HOLD_ACTION_DATA_20:
            actionData[20] = (uint8_t)(value & 0xFF);
            break;
        case HOLD_ACTION_DATA_21:
            actionData[21] = (uint8_t)(value & 0xFF);
            break;
        case HOLD_ACTION_DATA_22:
            actionData[22] = (uint8_t)(value & 0xFF);
            break;
        case HOLD_ACTION_DATA_23:
            actionData[23] = (uint8_t)(value & 0xFF);
            break;
        case HOLD_ACTION_DATA_24:
            actionData[24] = (uint8_t)(value & 0xFF);
            break;
        case HOLD_ACTION_DATA_25:
            actionData[25] = (uint8_t)(value & 0xFF);
            break;
        case HOLD_ACTION_DATA_26:
            actionData[26] = (uint8_t)(value & 0xFF);
            break;
        case HOLD_ACTION_DATA_27:
            actionData[27] = (uint8_t)(value & 0xFF);
            break;
        case HOLD_ACTION_DATA_28:
            actionData[28] = (uint8_t)(value & 0xFF);
            break;
        case HOLD_ACTION_DATA_29:
            actionData[29] = (uint8_t)(value & 0xFF);
            break;
        case HOLD_ACTION_DATA_30:
            actionData[30] = (uint8_t)(value & 0xFF);
            break;
        case HOLD_ACTION_DATA_31:
            actionData[31] = (uint8_t)(value & 0xFF);
            break;
    }
}

void handleActionCode(int actionCode) {
    switch (actionCode) {
        case ACTION_CODE_SET_MAC:
            EEPROM.write(EEPROM_ADDRESS_MAC_1, actionData[0]);
            EEPROM.write(EEPROM_ADDRESS_MAC_2, actionData[1]);
            EEPROM.write(EEPROM_ADDRESS_MAC_3, actionData[2]);
            EEPROM.write(EEPROM_ADDRESS_MAC_4, actionData[3]);
            EEPROM.write(EEPROM_ADDRESS_MAC_5, actionData[4]);
            EEPROM.write(EEPROM_ADDRESS_MAC_6, actionData[5]);
            break;

        case ACTION_CODE_SET_SLAVE_ID_BY_MAC:
            {
                byte targetMAC[6];
                targetMAC[0] = actionData[0];
                targetMAC[1] = actionData[1];
                targetMAC[2] = actionData[2];
                targetMAC[3] = actionData[3];
                targetMAC[4] = actionData[4];
                targetMAC[5] = actionData[5];
                byte newSlaveId = actionData[6];

                if (targetMAC[0] == EEPROM.read(EEPROM_ADDRESS_MAC_1) &&
                    targetMAC[1] == EEPROM.read(EEPROM_ADDRESS_MAC_2) &&
                    targetMAC[2] == EEPROM.read(EEPROM_ADDRESS_MAC_3) &&
                    targetMAC[3] == EEPROM.read(EEPROM_ADDRESS_MAC_4) &&
                    targetMAC[4] == EEPROM.read(EEPROM_ADDRESS_MAC_5) &&
                    targetMAC[5] == EEPROM.read(EEPROM_ADDRESS_MAC_6)) {
                    if (newSlaveId >= 1 && newSlaveId <= 247) {
                        EEPROM.write(EEPROM_ADDRESS_SLAVE_ID, newSlaveId);
                        addr = newSlaveId;
                    }
                }
            }
            break;

        case ACTION_CODE_SET_TIME:
            currentHour = actionData[0];
            currentMinute = actionData[1];
            currentSecond = actionData[2];
            break;

        case ACTION_CODE_SET_DATE:
            currentDay = actionData[0];
            currentMonth = actionData[1];
            currentYear = actionData[2];
            break;

        case ACTION_CODE_SET_NAME:
            for (int i = 0; i < NAME_SIZE; i++) {
                EEPROM.write(EEPROM_ADDRESS_NAME_0 + i, actionData[i]);
            }
            break;

        case ACTION_CODE_SET_SCENARIO:
            ScenarioManager::writeScenarioFromRawBytes(actionData[0], &actionData[1]);
            break;

        case ACTION_CODE_GET_SCENARIO:
            {
                uint8_t scenarioIndex = actionData[0];
                if (ScenarioManager::readScenarioToRawBytes(scenarioIndex, &actionData[1])) {
                    actionData[0] = scenarioIndex;
                }
            }
            break;

        case ACTION_CODE_GET_MAC:
            clearActionData();
            actionData[0] = EEPROM.read(EEPROM_ADDRESS_MAC_1);
            actionData[1] = EEPROM.read(EEPROM_ADDRESS_MAC_2);
            actionData[2] = EEPROM.read(EEPROM_ADDRESS_MAC_3);
            actionData[3] = EEPROM.read(EEPROM_ADDRESS_MAC_4);
            actionData[4] = EEPROM.read(EEPROM_ADDRESS_MAC_5);
            actionData[5] = EEPROM.read(EEPROM_ADDRESS_MAC_6);
            break;

        case ACTION_CODE_GET_TIME:
            clearActionData();
            actionData[0] = currentHour;
            actionData[1] = currentMinute;
            actionData[2] = currentSecond;
            break;

        case ACTION_CODE_GET_DATE:
            clearActionData();
            actionData[0] = currentDay;
            actionData[1] = currentMonth;
            actionData[2] = currentYear;
            break;

        case ACTION_CODE_GET_NAME:
            clearActionData();
            for (int i = 0; i < NAME_SIZE; i++) {
                actionData[i] = EEPROM.read(EEPROM_ADDRESS_NAME_0 + i);
            }
            break;

        case ACTION_CODE_SET_SENSOR_CONFIG:
            for (uint8_t i = 0; i < (sizeof(sensorPins) / sizeof(sensorPins[0])); i++) {
                sensorIds[i] = actionData[i];
                EEPROM.write(EEPROM_ADDRESS_SENSOR_ID_0 + i, sensorIds[i]);
            }
            updateConfiguredIoStates();
            break;

        case ACTION_CODE_GET_SENSOR_CONFIG:
            clearActionData();
            for (uint8_t i = 0; i < (sizeof(sensorPins) / sizeof(sensorPins[0])); i++) {
                actionData[i] = sensorIds[i];
            }
            break;

        case ACTION_CODE_SET_ACTUATOR_CONFIG:
            for (uint8_t i = 0; i < (sizeof(actuatorPins) / sizeof(actuatorPins[0])); i++) {
                actuatorIds[i] = actionData[i];
                EEPROM.write(EEPROM_ADDRESS_ACTUATOR_ID_0 + i, actuatorIds[i]);
                if (!isActuatorConfigured(i) && !actuatorCoilForceOn[i]) {
                    actuatorStates[i] = 0;
                    digitalWrite(actuatorPins[i], LOW);
                }
            }
            applyConfiguredActuatorStates();
            break;

        case ACTION_CODE_GET_ACTUATOR_CONFIG:
            clearActionData();
            for (uint8_t i = 0; i < (sizeof(actuatorPins) / sizeof(actuatorPins[0])); i++) {
                actionData[i] = actuatorIds[i];
            }
            break;

        case ACTION_CODE_CLEAR_ALL_SCENARIOS:
            ScenarioManager::clearAllScenarios();
            break;

        case ACTION_CODE_SENSOR_STATE_CHANGED:
            processRemoteSensorStateChange(actionData[0], actionData[1]);
            break;

        case ACTION_CODE_SET_BRIGHTNESS:
            lightController.applyBrightnessValue(actionData[0]);
            break;

        case ACTION_CODE_GET_CURRENT_BRIGHTNESS:
            memset(actionData, 0, ACTION_DATA_SIZE);
            actionData[0] = getCurrentBrightnessPercent();
            break;

        default:
            break;
    }
}

ISR(TIMER2_COMPA_vect) {
    lightController.onTimerTick();
}

void tickTimer() {
    // Внешнее управление не сбрасываем по таймауту.
    
    // Обновление времени
    currentSecond++;
    
    if (currentSecond >= 60) {
        currentSecond = 0;
        currentMinute++;
        
        if (currentMinute >= 60) {
            currentMinute = 0;
            currentHour++;
            
            if (currentHour >= 24) {
                currentHour = 0;
                // Следующий день
                currentDay++;
                
                // Определяем количество дней в текущем месяце
                byte daysInMonth;
                
                switch (currentMonth) {
                    case 1:  // Январь
                    case 3:  // Март
                    case 5:  // Май
                    case 7:  // Июль
                    case 8:  // Август
                    case 10: // Октябрь
                    case 12: // Декабрь
                        daysInMonth = 31;
                        break;
                        
                    case 4:  // Апрель
                    case 6:  // Июнь
                    case 9:  // Сентябрь
                    case 11: // Ноябрь
                        daysInMonth = 30;
                        break;
                        
                    case 2:  // Февраль
                        // Проверка на високосный год
                        if (isLeapYear(currentYear)) {
                            daysInMonth = 29;
                        } else {
                            daysInMonth = 28;
                        }
                        break;
                        
                    default:
                        daysInMonth = 31;
                        break;
                }
                
                // Проверка перехода на следующий месяц
                if (currentDay > daysInMonth) {
                    currentDay = 1;
                    currentMonth++;
                    
                    if (currentMonth > 12) {
                        currentMonth = 1;
                        currentYear++;
                        // Ограничение года (00-99)
                        if (currentYear > 99) {
                            currentYear = 0;
                        }
                    }
                }
            }
        }
    }
}

// Функция проверки високосного года для диапазона 2000-2099
bool isLeapYear(byte year) {
    // year хранит последние две цифры года (например, 24 для 2024)
    // Правило: год високосный, если делится на 4
    // Исключение: 1900 не високосный, но наш диапазон только 2000-2099
    // 2000 год (year=0) был високосным
    return (year % 4 == 0);
}

// =====================================================
// Вспомогательные функции
// =====================================================

void Clear_buff() {
    for (int i = 0; i < LEN; i++) {
        buffSerial[i] = 0;
    }
    count_rx = 0;
    flag_rx = 0;
}