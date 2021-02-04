#ifndef __CRC_H__
#define __ICRC_H__

#include <cstdint>

namespace CRC {

/**
 * @brief Calculates the CRC32 of a buffer
 * @param data Pointer to the buffer
 * @param length Length of buffer in bytes
 * @return CRC32
 */
uint32_t calculateCRC32(const void* data, uint32_t length);

/**
 * @brief Calculates the CRC8 of a buffer
 * @param data Pointer to the buffer
 * @param length Length of buffer in bytes
 * @return CRC8
 */
uint8_t calculateCRC8(const void* data, uint32_t length);

}; // namespace CRC

#endif // __CRC_H__
