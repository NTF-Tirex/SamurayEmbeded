#include "scenario_manager.h"

namespace {
    const uint8_t CRC8_POLY = 0x07;

    uint8_t crc8_update(uint8_t crc, uint8_t data) {
        crc ^= data;
        for (uint8_t i = 0; i < 8; i++) {
            if (crc & 0x80) {
                crc = (uint8_t)((crc << 1) ^ CRC8_POLY);
            } else {
                crc <<= 1;
            }
        }
        return crc;
    }

    uint8_t calcBytesCrc(const uint8_t *data, uint8_t len) {
        uint8_t crc = 0;
        for (uint8_t i = 0; i < len; i++) {
            crc = crc8_update(crc, data[i]);
        }
        return crc;
    }

    bool isReservedAddress(uint8_t address) {
        return address == 0 || address == 1 || address == 127;
    }

    bool isValidSourceAddress(uint8_t address) {
        return address >= 2 && address <= 126;
    }

    uint8_t *recordBytes(ScenarioRecord &record) {
        return reinterpret_cast<uint8_t *>(&record);
    }

    const uint8_t *recordBytes(const ScenarioRecord &record) {
        return reinterpret_cast<const uint8_t *>(&record);
    }
}

namespace ScenarioManager {
    void begin() {
        uint8_t count = EEPROM.read(EEPROM_ADDRESS_SCENARIO_COUNT);
        if (count > SCENARIO_MAX_COUNT) {
            EEPROM.write(EEPROM_ADDRESS_SCENARIO_COUNT, 0);
            EEPROM.write(EEPROM_ADDRESS_SCENARIO_CRC_ALL, 0);
        }
    }

    uint8_t getScenarioCount() {
        uint8_t count = EEPROM.read(EEPROM_ADDRESS_SCENARIO_COUNT);
        return (count <= SCENARIO_MAX_COUNT) ? count : 0;
    }

    bool setScenarioCount(uint8_t count) {
        if (count > SCENARIO_MAX_COUNT) {
            return false;
        }
        EEPROM.write(EEPROM_ADDRESS_SCENARIO_COUNT, count);
        return true;
    }

    uint16_t getScenarioEepromAddress(uint8_t index) {
        return (uint16_t)(EEPROM_ADDRESS_SCENARIO_FIRST + ((uint16_t)index * SCENARIO_SIZE));
    }

    bool isValidIndex(uint8_t index) {
        return index < SCENARIO_MAX_COUNT;
    }

    bool readScenario(uint8_t index, ScenarioRecord &record) {
        if (!isValidIndex(index)) {
            return false;
        }

        uint16_t eepromAddress = getScenarioEepromAddress(index);
        uint8_t *bytes = recordBytes(record);
        for (uint8_t i = 0; i < SCENARIO_SIZE; i++) {
            bytes[i] = EEPROM.read(eepromAddress + i);
        }
        return true;
    }

    bool writeScenario(uint8_t index, ScenarioRecord &record) {
        if (!isValidIndex(index)) {
            return false;
        }

        if (!isRecordStructValid(record)) {
            return false;
        }

        updateRecordCrc(record);

        uint16_t eepromAddress = getScenarioEepromAddress(index);
        const uint8_t *bytes = recordBytes(record);
        for (uint8_t i = 0; i < SCENARIO_SIZE; i++) {
            EEPROM.write(eepromAddress + i, bytes[i]);
        }

        uint8_t count = getScenarioCount();
        if (index >= count) {
            setScenarioCount(index + 1);
        }

        updateAllScenariosCrc();
        return true;
    }
    bool readScenarioToRawBytes(uint8_t index, uint8_t *rawBytes) {
        if (!isValidIndex(index) || rawBytes == nullptr) {
            return false;
        }

        ScenarioRecord record;
        if (!readScenario(index, record)) {
            return false;
        }

        const uint8_t *bytes = recordBytes(record);
        for (uint8_t i = 0; i < SCENARIO_SIZE; i++) {
            rawBytes[i] = bytes[i];
        }

        return true;
    }

    bool writeScenarioFromRawBytes(uint8_t index, const uint8_t *rawBytes) {
        if (!isValidIndex(index) || rawBytes == nullptr) {
            return false;
        }

        ScenarioRecord record;
        uint8_t *bytes = recordBytes(record);
        for (uint8_t i = 0; i < SCENARIO_SIZE; i++) {
            bytes[i] = rawBytes[i];
        }

        if (!isRecordStructValid(record)) {
            return false;
        }

        if (!isRecordCrcValid(record)) {
            return false;
        }

        return writeScenario(index, record);
    }


    bool disableScenario(uint8_t index) {
        if (!isValidIndex(index)) {
            return false;
        }

        ScenarioRecord record;
        if (!readScenario(index, record)) {
            return false;
        }

        if (isSlotEmpty(record)) {
            return false;
        }

        record.sourceAddress = SCENARIO_SOURCE_DISABLED;
        updateRecordCrc(record);

        uint16_t eepromAddress = getScenarioEepromAddress(index);
        const uint8_t *bytes = recordBytes(record);
        for (uint8_t i = 0; i < SCENARIO_SIZE; i++) {
            EEPROM.write(eepromAddress + i, bytes[i]);
        }

        updateAllScenariosCrc();
        return true;
    }

    bool clearScenario(uint8_t index) {
        if (!isValidIndex(index)) {
            return false;
        }

        ScenarioRecord record;
        makeEmptyRecord(record);

        uint16_t eepromAddress = getScenarioEepromAddress(index);
        const uint8_t *bytes = recordBytes(record);
        for (uint8_t i = 0; i < SCENARIO_SIZE; i++) {
            EEPROM.write(eepromAddress + i, bytes[i]);
        }

        uint8_t count = getScenarioCount();
        if (index + 1 == count) {
            while (count > 0) {
                ScenarioRecord tailRecord;
                if (!readScenario(count - 1, tailRecord)) {
                    break;
                }
                if (!isSlotEmpty(tailRecord)) {
                    break;
                }
                count--;
            }
            setScenarioCount(count);
        }

        updateAllScenariosCrc();
        return true;
    }

    bool clearAllScenarios() {
        ScenarioRecord emptyRecord;
        makeEmptyRecord(emptyRecord);
        const uint8_t *bytes = recordBytes(emptyRecord);

        for (uint8_t index = 0; index < SCENARIO_MAX_COUNT; index++) {
            uint16_t eepromAddress = getScenarioEepromAddress(index);
            for (uint8_t i = 0; i < SCENARIO_SIZE; i++) {
                EEPROM.write(eepromAddress + i, bytes[i]);
            }
        }

        EEPROM.write(EEPROM_ADDRESS_SCENARIO_COUNT, 0);
        EEPROM.write(EEPROM_ADDRESS_SCENARIO_CRC_ALL, 0);
        return true;
    }

    bool isSlotEmpty(const ScenarioRecord &record) {
        return record.sourceAddress == SCENARIO_SOURCE_EMPTY;
    }

    bool isSlotDisabled(const ScenarioRecord &record) {
        return record.sourceAddress == SCENARIO_SOURCE_DISABLED;
    }

    bool isSlotActive(const ScenarioRecord &record) {
        return isValidSourceAddress(record.sourceAddress);
    }

    bool isRecordStructValid(const ScenarioRecord &record) {
        if (!(isSlotEmpty(record) || isSlotDisabled(record) || isSlotActive(record))) {
            return false;
        }

        if (getSensorIndex(record.triggerByte) > 7) {
            return false;
        }

        if (getActuatorIndex(record.reactionByte) > 7) {
            return false;
        }

        if (getReactionType(record.reactionByte) > SCENARIO_REACTION_OUT) {
            return false;
        }

        if (hasTargetAddress(record, 0) || hasTargetAddress(record, 1) || hasTargetAddress(record, 127)) {
            return false;
        }

        return true;
    }

    bool isRecordCrcValid(const ScenarioRecord &record) {
        return calcRecordCrc(record) == record.crc;
    }

    uint8_t makeTriggerByte(uint8_t sensorIndex, uint8_t triggerValue) {
        return (uint8_t)(((sensorIndex & 0x07) << 5) | (triggerValue & 0x1F));
    }

    uint8_t getSensorIndex(uint8_t triggerByte) {
        return (triggerByte >> 5) & 0x07;
    }

    uint8_t getTriggerValue(uint8_t triggerByte) {
        return triggerByte & 0x1F;
    }

    uint8_t makeReactionByte(uint8_t reactionType, uint8_t actuatorIndex) {
        return (uint8_t)(((reactionType & 0x0F) << 4) | (actuatorIndex & 0x0F));
    }

    uint8_t getReactionType(uint8_t reactionByte) {
        return (reactionByte >> 4) & 0x0F;
    }

    uint8_t getActuatorIndex(uint8_t reactionByte) {
        return reactionByte & 0x0F;
    }

    bool setTargetAddress(ScenarioRecord &record, uint8_t address) {
        if (isReservedAddress(address)) {
            return false;
        }

        uint8_t byteIndex = address >> 3;
        uint8_t bitMask = (uint8_t)(1U << (address & 0x07));
        record.targetMask[byteIndex] |= bitMask;
        return true;
    }

    bool clearTargetAddress(ScenarioRecord &record, uint8_t address) {
        if (address > 127) {
            return false;
        }

        uint8_t byteIndex = address >> 3;
        uint8_t bitMask = (uint8_t)(1U << (address & 0x07));
        record.targetMask[byteIndex] &= (uint8_t)(~bitMask);
        return true;
    }

    bool hasTargetAddress(const ScenarioRecord &record, uint8_t address) {
        if (address > 127) {
            return false;
        }

        uint8_t byteIndex = address >> 3;
        uint8_t bitMask = (uint8_t)(1U << (address & 0x07));
        return (record.targetMask[byteIndex] & bitMask) != 0;
    }

    void clearTargetMask(ScenarioRecord &record) {
        for (uint8_t i = 0; i < SCENARIO_TARGET_MASK_SIZE; i++) {
            record.targetMask[i] = 0;
        }
    }

    void updateRecordCrc(ScenarioRecord &record) {
        record.crc = calcRecordCrc(record);
    }

    uint8_t calcRecordCrc(const ScenarioRecord &record) {
        return calcBytesCrc(recordBytes(record), SCENARIO_SIZE - 1);
    }

    uint8_t calcAllScenariosCrc(uint8_t count) {
        if (count > SCENARIO_MAX_COUNT) {
            count = SCENARIO_MAX_COUNT;
        }

        uint8_t crc = 0;
        for (uint8_t index = 0; index < count; index++) {
            uint16_t eepromAddress = getScenarioEepromAddress(index);
            for (uint8_t i = 0; i < SCENARIO_SIZE; i++) {
                crc = crc8_update(crc, EEPROM.read(eepromAddress + i));
            }
        }
        return crc;
    }

    uint8_t calcStoredScenariosCrc() {
        return calcAllScenariosCrc(getScenarioCount());
    }

    void updateAllScenariosCrc() {
        EEPROM.write(EEPROM_ADDRESS_SCENARIO_CRC_ALL, calcStoredScenariosCrc());
    }

    bool isAllScenariosCrcValid() {
        uint8_t storedCrc = EEPROM.read(EEPROM_ADDRESS_SCENARIO_CRC_ALL);
        uint8_t actualCrc = calcStoredScenariosCrc();
        return storedCrc == actualCrc;
    }

    void makeEmptyRecord(ScenarioRecord &record) {
        record.sourceAddress = SCENARIO_SOURCE_EMPTY;
        record.triggerByte = 0;
        clearTargetMask(record);
        record.reactionByte = 0;
        record.reactionValue = 0;
        updateRecordCrc(record);
    }

    void makeDisabledRecord(ScenarioRecord &record) {
        record.sourceAddress = SCENARIO_SOURCE_DISABLED;
        record.triggerByte = 0;
        clearTargetMask(record);
        record.reactionByte = 0;
        record.reactionValue = 0;
        updateRecordCrc(record);
    }
}

