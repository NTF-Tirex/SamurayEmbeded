#pragma once

#include <Arduino.h>
#include <EEPROM.h>

// -----------------------------------------------------
// EEPROM layout for scenarios
// -----------------------------------------------------
#define EEPROM_ADDRESS_SCENARIO_COUNT     99
#define EEPROM_ADDRESS_SCENARIO_CRC_ALL   100
#define EEPROM_ADDRESS_SCENARIO_FIRST     101

// -----------------------------------------------------
// Scenario format
// 0  - sourceAddress
// 1  - sensorID
// 2  - sensorValue
// 3  - 18            targetMask[16] for addresses 0..127
// 19 - reactionByte  (bits 7..4 reactionType, bits 3..0 actuatorIndex)
// 20 - reactionValue
// 21 - crc
// -----------------------------------------------------
#define SCENARIO_SIZE                     22
#define SCENARIO_MAX_COUNT                40
#define SCENARIO_TARGET_MASK_SIZE         16

#define SCENARIO_SOURCE_EMPTY             0
#define SCENARIO_SOURCE_DISABLED          1
#define SCENARIO_SOURCE_RESERVED          127

// Reaction types
#define SCENARIO_REACTION_NONE            0
#define SCENARIO_REACTION_LUM             1
#define SCENARIO_REACTION_MODE            2
#define SCENARIO_REACTION_TIME            3
#define SCENARIO_REACTION_OUT             4

struct ScenarioRecord {
    uint8_t sourceAddress;
    uint8_t sensorID;
    uint8_t sensorValue;
    uint8_t targetMask[SCENARIO_TARGET_MASK_SIZE];
    uint8_t reactionByte;
    uint8_t reactionValue;
    uint8_t crc;
};

namespace ScenarioManager {
    void begin();

    uint8_t getScenarioCount();
    bool setScenarioCount(uint8_t count);

    uint16_t getScenarioEepromAddress(uint8_t index);
    bool isValidIndex(uint8_t index);

    bool readScenario(uint8_t index, ScenarioRecord &record);
    bool writeScenario(uint8_t index, ScenarioRecord &record);
    bool readScenarioToRawBytes(uint8_t index, uint8_t *rawBytes);
    bool writeScenarioFromRawBytes(uint8_t index, const uint8_t *rawBytes);

    bool disableScenario(uint8_t index);
    bool clearScenario(uint8_t index);
    bool clearAllScenarios();

    bool isSlotEmpty(const ScenarioRecord &record);
    bool isSlotDisabled(const ScenarioRecord &record);
    bool isSlotActive(const ScenarioRecord &record);
    bool isRecordStructValid(const ScenarioRecord &record);
    bool isRecordCrcValid(const ScenarioRecord &record);

    uint8_t makeReactionByte(uint8_t reactionType, uint8_t actuatorIndex);
    uint8_t getReactionType(uint8_t reactionByte);
    uint8_t getActuatorIndex(uint8_t reactionByte);

    bool setTargetAddress(ScenarioRecord &record, uint8_t address);
    bool clearTargetAddress(ScenarioRecord &record, uint8_t address);
    bool hasTargetAddress(const ScenarioRecord &record, uint8_t address);
    void clearTargetMask(ScenarioRecord &record);

    void updateRecordCrc(ScenarioRecord &record);
    uint8_t calcRecordCrc(const ScenarioRecord &record);

    uint8_t calcAllScenariosCrc(uint8_t count);
    uint8_t calcStoredScenariosCrc();
    void updateAllScenariosCrc();
    bool isAllScenariosCrcValid();

    void makeEmptyRecord(ScenarioRecord &record);
    void makeDisabledRecord(ScenarioRecord &record);
}

