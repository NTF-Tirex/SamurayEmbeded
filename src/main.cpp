#include <Arduino.h>
#include "GyverTimers.h"
#include <microDS18B20/microDS18B20.h>
#include <EEPROM.h>

/* Определение выводов */
#define DS18B20_DATA        2   // датчик температуры (1 wire)
#define ON_OFF_RELAY_1      3   // управление оптореле №1 
#define ON_OFF_RELAY_2      4   // управление оптореле №2 
#define MODE_TURN           5   // контроль тревожного выхода
#define ONOFF_TURN          6   // контроль датчика холла
#define PWM_PIN             7   // вывод ШИМ

#define ADDR_BIT_6          11  // адрес RS485 Bit6
#define ADDR_BIT_5          10  // адрес RS485 Bit5
#define ADDR_BIT_4          12  // адрес RS485 Bit4
#define ADDR_BIT_3          9   // адрес RS485 Bit3
#define ADDR_BIT_2          17  // адрес RS485 Bit2
#define ADDR_BIT_1          8   // адрес RS485 Bit1
#define ADDR_BIT_0          16  // адрес RS485 Bit0

//----------------------------------------------  
// Регистры Modbus
//----------------------------------------------  

// Регистры флагов (Coils)
#define COIL_MODE_CURRENT       0
#define COIL_ONOFF              1

// Дискретные входы (Discrete Inputs)
#define INPUT_DISCRETE_MODE         0  
#define INPUT_DISCRETE_ONOFF_TURN   1 

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
#define INPUT_MAC_ENABLED           9
#define INPUT_MAC_BYTE_1            10
#define INPUT_MAC_BYTE_2            11
#define INPUT_MAC_BYTE_3            12
#define INPUT_MAC_BYTE_4            13
#define INPUT_MAC_BYTE_5            14
#define INPUT_MAC_BYTE_6            15
#define INPUT_CALENDAR_ENABLED      16
#define INPUT_DATE_DAY              17
#define INPUT_DATE_MONTH            18
#define INPUT_DATE_YEAR             19
#define INPUT_TIME_HOUR             20
#define INPUT_DATE_MINITES          21
#define INPUT_DATE_SECONDS          22
#define INPUT_NAME_ENABLED          23  // Флаг наличия имени
#define INPUT_NAME_BYTE_0           24  // Байты имени (первые 32 символа)
#define INPUT_NAME_BYTE_1           25
#define INPUT_NAME_BYTE_2           26
#define INPUT_NAME_BYTE_3           27
#define INPUT_NAME_BYTE_4           28
#define INPUT_NAME_BYTE_5           29
#define INPUT_NAME_BYTE_6           30
#define INPUT_NAME_BYTE_7           31
#define INPUT_NAME_BYTE_8           32
#define INPUT_NAME_BYTE_9           33
#define INPUT_NAME_BYTE_10          34
#define INPUT_NAME_BYTE_11          35
#define INPUT_NAME_BYTE_12          36
#define INPUT_NAME_BYTE_13          37
#define INPUT_NAME_BYTE_14          38
#define INPUT_NAME_BYTE_15          39
#define INPUT_NAME_BYTE_16          40
#define INPUT_NAME_BYTE_17          41
#define INPUT_NAME_BYTE_18          42
#define INPUT_NAME_BYTE_19          43
#define INPUT_NAME_BYTE_20          44
#define INPUT_NAME_BYTE_21          45
#define INPUT_NAME_BYTE_22          46
#define INPUT_NAME_BYTE_23          47
#define INPUT_NAME_BYTE_24          48
#define INPUT_NAME_BYTE_25          49
#define INPUT_NAME_BYTE_26          50
#define INPUT_NAME_BYTE_27          51
#define INPUT_NAME_BYTE_28          52
#define INPUT_NAME_BYTE_29          53
#define INPUT_NAME_BYTE_30          54
#define INPUT_NAME_BYTE_31          55

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

// Адреса в EEPROM (только для сохраняемых данных)
#define EEPROM_ADDRESS_SLAVE_ID         0
#define EEPROM_ADDRESS_MODE_CURRENT     1
#define EEPROM_ADDRESS_ONOFF            2 
#define EEPROM_ADDRESS_LEVEL_MODE       3
#define EEPROM_ADDRESS_MAC_1            4
#define EEPROM_ADDRESS_MAC_2            5
#define EEPROM_ADDRESS_MAC_3            6
#define EEPROM_ADDRESS_MAC_4            7
#define EEPROM_ADDRESS_MAC_5            8
#define EEPROM_ADDRESS_MAC_6            9
#define EEPROM_ADDRESS_NAME_0           10  // Имя устройства (32 байта)
#define EEPROM_ADDRESS_NAME_1           11
#define EEPROM_ADDRESS_NAME_2           12
#define EEPROM_ADDRESS_NAME_3           13
#define EEPROM_ADDRESS_NAME_4           14
#define EEPROM_ADDRESS_NAME_5           15
#define EEPROM_ADDRESS_NAME_6           16
#define EEPROM_ADDRESS_NAME_7           17
#define EEPROM_ADDRESS_NAME_8           18
#define EEPROM_ADDRESS_NAME_9           19
#define EEPROM_ADDRESS_NAME_10          20
#define EEPROM_ADDRESS_NAME_11          21
#define EEPROM_ADDRESS_NAME_12          22
#define EEPROM_ADDRESS_NAME_13          23
#define EEPROM_ADDRESS_NAME_14          24
#define EEPROM_ADDRESS_NAME_15          25
#define EEPROM_ADDRESS_NAME_16          26
#define EEPROM_ADDRESS_NAME_17          27
#define EEPROM_ADDRESS_NAME_18          28
#define EEPROM_ADDRESS_NAME_19          29
#define EEPROM_ADDRESS_NAME_20          30
#define EEPROM_ADDRESS_NAME_21          31
#define EEPROM_ADDRESS_NAME_22          32
#define EEPROM_ADDRESS_NAME_23          33
#define EEPROM_ADDRESS_NAME_24          34
#define EEPROM_ADDRESS_NAME_25          35
#define EEPROM_ADDRESS_NAME_26          36
#define EEPROM_ADDRESS_NAME_27          37
#define EEPROM_ADDRESS_NAME_28          38
#define EEPROM_ADDRESS_NAME_29          39
#define EEPROM_ADDRESS_NAME_30          40
#define EEPROM_ADDRESS_NAME_31          41

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
#define DEVICE_TYPE_ID              3       // например, фосфор
#define FIRMWARE_HIGH               1 
#define FIRMWARE_LOW                0
#define SERIAL_NUMBER               1
#define SLAVE_ID_DEFAULT            128
#define MIN_BRIGHTNESS              20      // минимальная яркость (0-100)
#define ACTION_DATA_SIZE            32      // размер массива данных действий
#define NAME_SIZE                   32      // размер имени устройства

const char base[16] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};

const int coilsList[] = { 
    COIL_MODE_CURRENT, 
    COIL_ONOFF
};

const int inputDiscreteList[] = { 
    INPUT_DISCRETE_MODE, 
    INPUT_DISCRETE_ONOFF_TURN 
};

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
    INPUT_MAC_ENABLED,
    INPUT_MAC_BYTE_1,
    INPUT_MAC_BYTE_2,
    INPUT_MAC_BYTE_3,
    INPUT_MAC_BYTE_4,
    INPUT_MAC_BYTE_5,
    INPUT_MAC_BYTE_6,
    INPUT_CALENDAR_ENABLED,
    INPUT_DATE_DAY,
    INPUT_DATE_MONTH,
    INPUT_DATE_YEAR,
    INPUT_TIME_HOUR,
    INPUT_DATE_MINITES,
    INPUT_DATE_SECONDS,
    INPUT_NAME_ENABLED,
    INPUT_NAME_BYTE_0,
    INPUT_NAME_BYTE_1,
    INPUT_NAME_BYTE_2,
    INPUT_NAME_BYTE_3,
    INPUT_NAME_BYTE_4,
    INPUT_NAME_BYTE_5,
    INPUT_NAME_BYTE_6,
    INPUT_NAME_BYTE_7,
    INPUT_NAME_BYTE_8,
    INPUT_NAME_BYTE_9,
    INPUT_NAME_BYTE_10,
    INPUT_NAME_BYTE_11,
    INPUT_NAME_BYTE_12,
    INPUT_NAME_BYTE_13,
    INPUT_NAME_BYTE_14,
    INPUT_NAME_BYTE_15,
    INPUT_NAME_BYTE_16,
    INPUT_NAME_BYTE_17,
    INPUT_NAME_BYTE_18,
    INPUT_NAME_BYTE_19,
    INPUT_NAME_BYTE_20,
    INPUT_NAME_BYTE_21,
    INPUT_NAME_BYTE_22,
    INPUT_NAME_BYTE_23,
    INPUT_NAME_BYTE_24,
    INPUT_NAME_BYTE_25,
    INPUT_NAME_BYTE_26,
    INPUT_NAME_BYTE_27,
    INPUT_NAME_BYTE_28,
    INPUT_NAME_BYTE_29,
    INPUT_NAME_BYTE_30,
    INPUT_NAME_BYTE_31
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
    EEPROM_ADDRESS_NAME_31
};

/* Глобальные переменные */
byte addr;
char buffSerial[LEN] {};
char RX = 0;
int count_rx = 0;
char flag_rx = 0;

// Массив для данных действий (вместо EEPROM)
byte actionData[ACTION_DATA_SIZE] = {0};

// Переменные для даты
byte currentDay = 0;
byte currentMonth = 0;
byte currentYear = 0;

// Переменные для времени
byte currentHour = 0;
byte currentMinute = 0;
byte currentSecond = 0;

/* ШИМ */
int duty = 0;
bool lightEnabled = false;
bool lightBrightness = false;

bool isExternalControl = false;
int countWithoutExternalControl = 0;
const int timeoutExternalControl = 10;

/* Датчик температуры */
MicroDS18B20<DS18B20_DATA> sensor;

/* Прототипы функций */
void getAddress();
void setupEEPROM();
void clearEEPROM();

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

void setBright(int percent);
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

void setup() {
    Serial.begin(9600);

    pinMode(ON_OFF_RELAY_1, OUTPUT);
    pinMode(ON_OFF_RELAY_2, OUTPUT);
    pinMode(MODE_TURN, INPUT);
    pinMode(ONOFF_TURN, INPUT);
    pinMode(PWM_PIN, OUTPUT);

    pinMode(ADDR_BIT_6, INPUT);  
    pinMode(ADDR_BIT_5, INPUT); 
    pinMode(ADDR_BIT_4, INPUT); 
    pinMode(ADDR_BIT_3, INPUT); 
    pinMode(ADDR_BIT_2, INPUT); 
    pinMode(ADDR_BIT_1, INPUT); 
    pinMode(ADDR_BIT_0, INPUT);  

    Timer2.setFrequency(10000UL);   // 10 кГц для ШИМ
    Timer2.enableISR();

    getAddress();
    setupEEPROM();
}

void loop() {
    byte extModeState = 0;
    byte extOnOffState = 0;
    byte level = 30;

    if (isExternalControl) {
        extModeState = !EEPROM.read(EEPROM_ADDRESS_MODE_CURRENT);
        extOnOffState = EEPROM.read(EEPROM_ADDRESS_ONOFF);
        level = EEPROM.read(EEPROM_ADDRESS_LEVEL_MODE);
    } else {
        extModeState = digitalRead(MODE_TURN); 
        extOnOffState = digitalRead(ONOFF_TURN); 
    }
   digitalWrite(ON_OFF_RELAY_1, extModeState);
//         digitalWrite(ON_OFF_RELAY_2, extModeState);
    if (extModeState == 0) {
        setBright(100);
    } else {
        setBright(level);
    }

    lightEnabled = (extOnOffState == 1);
    
    readPacketFromPort();
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
                
                int adu[LEN];
                int lenght = 0;
                hexStringToIntArray(buffSerial, adu, &lenght);
                
                verifyAduData(adu, lenght);
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
    if (data[0] != 0) {
        char packet[256];
        int j = 0;
        
        packet[j++] = ':';
        
        for (int i = 0; i < len; i++) {
            char hex[2];
            getHexFromHDec(data[i], hex);
            packet[j++] = hex[0];
            packet[j++] = hex[1];
        }
        
        packet[j++] = '\r';
        packet[j++] = '\n';
        packet[j] = '\0';
        
        Serial.print(packet);
    }
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
    int answer[5];
    for (int i = 0; i < 3; i++) {
        answer[i] = data[i];
    }
    int lrc = lrc8(answer, 4);
    answer[4] = lrc;

    sendPacketToPort(answer, 5);
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
    // Инициализация coils
    int coilLen = sizeof(coilsList) / sizeof(coilsList[0]);
    for (int i = 0; i < coilLen; i++) {
        byte value = EEPROM.read(eepromAddressList[i]);
        writeAndSetCoil(i, value);
    }
    
    // Инициализация массива actionData нулями
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

// =====================================================
// Чтение значений
// =====================================================

byte readCoilFromEEPROM(int index) {
    switch (coilsList[index]) {
        case COIL_MODE_CURRENT:     return EEPROM.read(EEPROM_ADDRESS_MODE_CURRENT);
        case COIL_ONOFF:            return EEPROM.read(EEPROM_ADDRESS_ONOFF);
        default:                    return 0;
    }
}

int readHoldRegisterFromEEPROM(int index) {
    switch (holdRegisterList[index]) {
        case HOLD_LEVEL_MODE:       return EEPROM.read(EEPROM_ADDRESS_LEVEL_MODE);
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
    switch (inputDiscreteList[index]) {
        case INPUT_DISCRETE_MODE:           return !digitalRead(MODE_TURN);
        case INPUT_DISCRETE_ONOFF_TURN:     return digitalRead(ONOFF_TURN);
        default:                            return 0;
    }
}

int readInputState(int index) {
    switch (inputStateList[index]) {
        case INPUT_DEVICE_TYPE_ID:      return DEVICE_TYPE_ID;
        case INPUT_SENSOR_TEMPERATURE:  
            sensor.requestTemp();
            return (int)sensor.getTemp();
        case INPUT_SERIAL_NUMBER:       return SERIAL_NUMBER;
        case INPUT_FIRMWARE_HIGH:       return FIRMWARE_HIGH;
        case INPUT_FIRMWARE_LOW:        return FIRMWARE_LOW;
        
        // Размеры массивов
        case INPUT_COUNT_COILS:         return sizeof(coilsList) / sizeof(coilsList[0]);
        case INPUT_COUNT_DISCRETE:      return sizeof(inputDiscreteList) / sizeof(inputDiscreteList[0]);
        case INPUT_COUNT_REGISTERS:     return sizeof(inputStateList) / sizeof(inputStateList[0]);
        case INPUT_COUNT_HOLD:          return sizeof(holdRegisterList) / sizeof(holdRegisterList[0]);
        
        // MAC адрес
        case INPUT_MAC_ENABLED:         
            // Проверяем, установлен ли MAC-адрес (не все байты равны 0)
            return (EEPROM.read(EEPROM_ADDRESS_MAC_1) != 0 || 
                    EEPROM.read(EEPROM_ADDRESS_MAC_2) != 0 || 
                    EEPROM.read(EEPROM_ADDRESS_MAC_3) != 0 || 
                    EEPROM.read(EEPROM_ADDRESS_MAC_4) != 0 || 
                    EEPROM.read(EEPROM_ADDRESS_MAC_5) != 0 || 
                    EEPROM.read(EEPROM_ADDRESS_MAC_6) != 0) ? 1 : 0;
            
        case INPUT_MAC_BYTE_1:          return EEPROM.read(EEPROM_ADDRESS_MAC_1);
        case INPUT_MAC_BYTE_2:          return EEPROM.read(EEPROM_ADDRESS_MAC_2);
        case INPUT_MAC_BYTE_3:          return EEPROM.read(EEPROM_ADDRESS_MAC_3);
        case INPUT_MAC_BYTE_4:          return EEPROM.read(EEPROM_ADDRESS_MAC_4);
        case INPUT_MAC_BYTE_5:          return EEPROM.read(EEPROM_ADDRESS_MAC_5);
        case INPUT_MAC_BYTE_6:          return EEPROM.read(EEPROM_ADDRESS_MAC_6);
        
        // Календарь
        case INPUT_CALENDAR_ENABLED:    
            // Проверяем, установлена ли дата (не все поля равны 0)
            return (currentDay != 0 || currentMonth != 0 || currentYear != 0 || 
                    currentHour != 0 || currentMinute != 0 || currentSecond != 0) ? 1 : 0;
        
        // Дата
        case INPUT_DATE_DAY:            return currentDay;
        case INPUT_DATE_MONTH:          return currentMonth;
        case INPUT_DATE_YEAR:           return currentYear;
        
        // Время
        case INPUT_TIME_HOUR:           return currentHour;
        case INPUT_DATE_MINITES:        return currentMinute;
        case INPUT_DATE_SECONDS:        return currentSecond;
        
        // Имя устройства
        case INPUT_NAME_ENABLED:
            // Проверяем, установлено ли имя (не все байты равны 0)
            for (int i = 0; i < NAME_SIZE; i++) {
                if (EEPROM.read(EEPROM_ADDRESS_NAME_0 + i) != 0) {
                    return 1;
                }
            }
            return 0;
            
        case INPUT_NAME_BYTE_0:  return EEPROM.read(EEPROM_ADDRESS_NAME_0);
        case INPUT_NAME_BYTE_1:  return EEPROM.read(EEPROM_ADDRESS_NAME_1);
        case INPUT_NAME_BYTE_2:  return EEPROM.read(EEPROM_ADDRESS_NAME_2);
        case INPUT_NAME_BYTE_3:  return EEPROM.read(EEPROM_ADDRESS_NAME_3);
        case INPUT_NAME_BYTE_4:  return EEPROM.read(EEPROM_ADDRESS_NAME_4);
        case INPUT_NAME_BYTE_5:  return EEPROM.read(EEPROM_ADDRESS_NAME_5);
        case INPUT_NAME_BYTE_6:  return EEPROM.read(EEPROM_ADDRESS_NAME_6);
        case INPUT_NAME_BYTE_7:  return EEPROM.read(EEPROM_ADDRESS_NAME_7);
        case INPUT_NAME_BYTE_8:  return EEPROM.read(EEPROM_ADDRESS_NAME_8);
        case INPUT_NAME_BYTE_9:  return EEPROM.read(EEPROM_ADDRESS_NAME_9);
        case INPUT_NAME_BYTE_10: return EEPROM.read(EEPROM_ADDRESS_NAME_10);
        case INPUT_NAME_BYTE_11: return EEPROM.read(EEPROM_ADDRESS_NAME_11);
        case INPUT_NAME_BYTE_12: return EEPROM.read(EEPROM_ADDRESS_NAME_12);
        case INPUT_NAME_BYTE_13: return EEPROM.read(EEPROM_ADDRESS_NAME_13);
        case INPUT_NAME_BYTE_14: return EEPROM.read(EEPROM_ADDRESS_NAME_14);
        case INPUT_NAME_BYTE_15: return EEPROM.read(EEPROM_ADDRESS_NAME_15);
        case INPUT_NAME_BYTE_16: return EEPROM.read(EEPROM_ADDRESS_NAME_16);
        case INPUT_NAME_BYTE_17: return EEPROM.read(EEPROM_ADDRESS_NAME_17);
        case INPUT_NAME_BYTE_18: return EEPROM.read(EEPROM_ADDRESS_NAME_18);
        case INPUT_NAME_BYTE_19: return EEPROM.read(EEPROM_ADDRESS_NAME_19);
        case INPUT_NAME_BYTE_20: return EEPROM.read(EEPROM_ADDRESS_NAME_20);
        case INPUT_NAME_BYTE_21: return EEPROM.read(EEPROM_ADDRESS_NAME_21);
        case INPUT_NAME_BYTE_22: return EEPROM.read(EEPROM_ADDRESS_NAME_22);
        case INPUT_NAME_BYTE_23: return EEPROM.read(EEPROM_ADDRESS_NAME_23);
        case INPUT_NAME_BYTE_24: return EEPROM.read(EEPROM_ADDRESS_NAME_24);
        case INPUT_NAME_BYTE_25: return EEPROM.read(EEPROM_ADDRESS_NAME_25);
        case INPUT_NAME_BYTE_26: return EEPROM.read(EEPROM_ADDRESS_NAME_26);
        case INPUT_NAME_BYTE_27: return EEPROM.read(EEPROM_ADDRESS_NAME_27);
        case INPUT_NAME_BYTE_28: return EEPROM.read(EEPROM_ADDRESS_NAME_28);
        case INPUT_NAME_BYTE_29: return EEPROM.read(EEPROM_ADDRESS_NAME_29);
        case INPUT_NAME_BYTE_30: return EEPROM.read(EEPROM_ADDRESS_NAME_30);
        case INPUT_NAME_BYTE_31: return EEPROM.read(EEPROM_ADDRESS_NAME_31);
        
        default:                        return 0;
    }
}

// =====================================================
// Запись значений
// =====================================================

void writeAndSetCoil(int index, byte value) {
    switch (coilsList[index]) {
        case COIL_MODE_CURRENT:
            EEPROM.write(EEPROM_ADDRESS_MODE_CURRENT, value);
            break;
            
        case COIL_ONOFF:
            EEPROM.write(EEPROM_ADDRESS_ONOFF, value);
            break;
    }
}

void writeHoldRegister(int index, int value) {
    switch (holdRegisterList[index]) {
        case HOLD_LEVEL_MODE:
            EEPROM.write(EEPROM_ADDRESS_LEVEL_MODE, value);
            break;
        case HOLD_ACTION_CODE:
            handleActionCode(value);
            break;
        case HOLD_ACTION_DATA_0:
            actionData[0] = value;
            break;
        case HOLD_ACTION_DATA_1:
            actionData[1] = value;
            break;
        case HOLD_ACTION_DATA_2:
            actionData[2] = value;
            break;
        case HOLD_ACTION_DATA_3:
            actionData[3] = value;
            break;
        case HOLD_ACTION_DATA_4:
            actionData[4] = value;
            break;
        case HOLD_ACTION_DATA_5:
            actionData[5] = value;
            break;
        case HOLD_ACTION_DATA_6:
            actionData[6] = value;
            break;
        case HOLD_ACTION_DATA_7:
            actionData[7] = value;
            break;
        case HOLD_ACTION_DATA_8:
            actionData[8] = value;
            break;
        case HOLD_ACTION_DATA_9:
            actionData[9] = value;
            break;
        case HOLD_ACTION_DATA_10:
            actionData[10] = value;
            break;
        case HOLD_ACTION_DATA_11:
            actionData[11] = value;
            break;
        case HOLD_ACTION_DATA_12:
            actionData[12] = value;
            break;
        case HOLD_ACTION_DATA_13:
            actionData[13] = value;
            break;
        case HOLD_ACTION_DATA_14:
            actionData[14] = value;
            break;
        case HOLD_ACTION_DATA_15:
            actionData[15] = value;
            break;
        case HOLD_ACTION_DATA_16:
            actionData[16] = value;
            break;
        case HOLD_ACTION_DATA_17:
            actionData[17] = value;
            break;
        case HOLD_ACTION_DATA_18:
            actionData[18] = value;
            break;
        case HOLD_ACTION_DATA_19:
            actionData[19] = value;
            break;
        case HOLD_ACTION_DATA_20:
            actionData[20] = value;
            break;
        case HOLD_ACTION_DATA_21:
            actionData[21] = value;
            break;
        case HOLD_ACTION_DATA_22:
            actionData[22] = value;
            break;
        case HOLD_ACTION_DATA_23:
            actionData[23] = value;
            break;
        case HOLD_ACTION_DATA_24:
            actionData[24] = value;
            break;
        case HOLD_ACTION_DATA_25:
            actionData[25] = value;
            break;
        case HOLD_ACTION_DATA_26:
            actionData[26] = value;
            break;
        case HOLD_ACTION_DATA_27:
            actionData[27] = value;
            break;
        case HOLD_ACTION_DATA_28:
            actionData[28] = value;
            break;
        case HOLD_ACTION_DATA_29:
            actionData[29] = value;
            break;
        case HOLD_ACTION_DATA_30:
            actionData[30] = value;
            break;
        case HOLD_ACTION_DATA_31:
            actionData[31] = value;
            break;
    }
}

void handleActionCode(int actionCode) {
    switch (actionCode) {
        case ACTION_CODE_SET_MAC:
            // Чтение MAC-адреса из actionData (6 байт)
            EEPROM.write(EEPROM_ADDRESS_MAC_1, actionData[0]);
            EEPROM.write(EEPROM_ADDRESS_MAC_2, actionData[1]);
            EEPROM.write(EEPROM_ADDRESS_MAC_3, actionData[2]);
            EEPROM.write(EEPROM_ADDRESS_MAC_4, actionData[3]);
            EEPROM.write(EEPROM_ADDRESS_MAC_5, actionData[4]);
            EEPROM.write(EEPROM_ADDRESS_MAC_6, actionData[5]);
            break;
            
        case ACTION_CODE_SET_SLAVE_ID_BY_MAC:
            {
                // Получаем MAC-адрес из actionData (6 байт)
                byte targetMAC[6];
                targetMAC[0] = actionData[0];
                targetMAC[1] = actionData[1];
                targetMAC[2] = actionData[2];
                targetMAC[3] = actionData[3];
                targetMAC[4] = actionData[4];
                targetMAC[5] = actionData[5];
                
                // Получаем новый Slave ID из actionData[6]
                byte newSlaveId = actionData[6];
                
                // Сравниваем с собственным MAC-адресом
                if (targetMAC[0] == EEPROM.read(EEPROM_ADDRESS_MAC_1) &&
                    targetMAC[1] == EEPROM.read(EEPROM_ADDRESS_MAC_2) &&
                    targetMAC[2] == EEPROM.read(EEPROM_ADDRESS_MAC_3) &&
                    targetMAC[3] == EEPROM.read(EEPROM_ADDRESS_MAC_4) &&
                    targetMAC[4] == EEPROM.read(EEPROM_ADDRESS_MAC_5) &&
                    targetMAC[5] == EEPROM.read(EEPROM_ADDRESS_MAC_6)) {
                    
                    // Это наш MAC-адрес - устанавливаем новый Slave ID
                    if (newSlaveId >= 1 && newSlaveId <= 247) {
                        EEPROM.write(EEPROM_ADDRESS_SLAVE_ID, newSlaveId);
                        addr = newSlaveId;
                    }
                }
                // Если MAC не совпадает - ничего не делаем
            }
            break;
            
        case ACTION_CODE_SET_TIME:
            // Установка времени (данные в actionData[0], actionData[1], actionData[2])
            currentHour = actionData[0];
            currentMinute = actionData[1];
            currentSecond = actionData[2];
            break;
            
        case ACTION_CODE_SET_DATE:
            // Установка даты (данные в actionData[0], actionData[1], actionData[2])
            currentDay = actionData[0];
            currentMonth = actionData[1];
            currentYear = actionData[2];
            break;
            
        case ACTION_CODE_SET_NAME:
            // Сохранение имени из actionData (32 байта)
            for (int i = 0; i < NAME_SIZE; i++) {
                EEPROM.write(EEPROM_ADDRESS_NAME_0 + i, actionData[i]);
            }
            break;
            
        default:
            break;
    }
}

// =====================================================
// Управление яркостью / ШИМ
// =====================================================

void setBright(int percent) {
    if (percent < 0) percent = 0;
    if (percent > 100) percent = 100;

    if (percent > 0) {
        percent = MIN_BRIGHTNESS + (100 - MIN_BRIGHTNESS) * percent / 100;
    }

    duty = (255 * percent) / 100;
}

ISR(TIMER2_COMPA_vect) {
    static uint8_t counter = 0;
    static uint32_t lightCounter = 0;

    lightBrightness = (counter < (duty / 2));

    if (lightEnabled && lightBrightness) {
        digitalWrite(PWM_PIN, HIGH);
    } else {
        digitalWrite(PWM_PIN, LOW);
    }

    counter++;
    lightCounter++;

    if (lightCounter >= 10000UL) {
        lightCounter = 0;
        tickTimer();
    }
}

void tickTimer() {
    countWithoutExternalControl++;
    isExternalControl = (countWithoutExternalControl < timeoutExternalControl);
    
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