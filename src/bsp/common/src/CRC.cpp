#include "common/CRC.h"

uint32_t CRC::calculateCRC32(const void* data, uint32_t length) {
    // TODO: implement CRC32 algorithm
    return UINT32_MAX;
}

// Source: https://www.pololu.com/docs/0J44/6.7.6
uint8_t CRC::calculateCRC8(const void* data, uint32_t length) {
    const uint8_t polynomial = 0x91;
    uint8_t crc = 0;
    auto* message = (uint8_t*)data;

    for (uint32_t i = 0; i < length; i++) {
        crc ^= message[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 1) {
                crc ^= polynomial;
            }
            crc >>= 1;
        }
    }
    return crc;
}