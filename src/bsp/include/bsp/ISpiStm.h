#ifndef __ISPISTM_H__
#define __ISPISTM_H__

#include "common/IProtobufStream.h"
#include <cstdint>

#define CRC32_SIZE (sizeof(uint32_t))
#define STM_SPI_MAX_MESSAGE_LENGTH (2048u - CRC32_SIZE)

class ISpiStm : public IProtobufStream {
  public:
    virtual ~ISpiStm() = default;

    /**
     * @brief Sends a buffer to the Stm via spi by appending with the length and the CRC32
     * @param buffer Pointer to the data to send
     * @param length Number of bytes to send
     * @return True if transfer started. False otherwise.
     */
    virtual bool send(const uint8_t* buffer, uint16_t length) = 0;

    /**
     * @brief Receive data from spi inside a buffer
     * @param data The buffer in which to store the data received
     * @param length The number of bytes to read
     * @return true if successful, false otherwise
     */
    virtual bool receive(uint8_t* data, uint16_t length) = 0;

    /**
     * @brief Checks if driver is already busy transmitting data.
     * @return True if in use. False otherwise
     */
    virtual bool isBusy() const = 0;

    /**
     * @brief Check if driver is connected
     * @return true if connection has been established, false otherwise
     */
    virtual bool isConnected() const = 0;
};

#endif // __ISPISTM_H__
