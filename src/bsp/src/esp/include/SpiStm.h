#ifndef __SPISTM_H__
#define __SPISTM_H__

#include "SpiHeader.h"
#include "bsp/ISpiStm.h"
#include "logger/ILogger.h"
#include <FreeRTOS.h>
#include <FreeRTOSConfig.h>
#include <array>
#include <driver/spi_slave.h>
#include <semphr.h>
#include <task.h>

class SpiStm : public ISpiStm {
  public:
    SpiStm(ILogger& logger);

    bool send(const uint8_t* buffer, uint16_t length) override;

    bool isBusy() const override;

    void execute();

  protected:
    ILogger& m_logger;
    enum class transmitState { SENDING_HEADER, SENDING_PAYLOAD, ERROR } m_txState;
    enum class receiveState {
        RECEIVING_HEADER,
        PARSING_HEADER,
        RECEIVING_PAYLOAD,
        VALIDATE_CRC,
        VALID_PAYLOAD,
        ERROR
    } m_rxState;

    spi_slave_transaction_t m_transaction;

    struct Message {
        // These buffers need to be word-alligned for DMA.
        alignas(32) std::array<uint8_t, STM_SPI_MAX_MESSAGE_LENGTH> m_data;
        uint32_t m_sizeBytes;
    } m_inboundMessage, m_outboundMessage;
    // Used for transaction
    alignas(32) StmSpi::Header m_outboundHeader;
    StmSpi::Header* m_inboundHeader;

    void updateOutboundHeader();

  private:
    static void notifyMaster();
    static void transactionCallback(void* instance, spi_slave_transaction_t* transaction);

    StaticSemaphore_t m_semaphoreBuffer;
    SemaphoreHandle_t m_semaphore;

    std::array<StackType_t, 4096> m_stackData;
    StaticTask_t m_stackBuffer;
    TaskHandle_t m_taskHandle;
    uint32_t m_loopRate;

    bool m_isBusy;
};
#endif // __SPISTM_H__
