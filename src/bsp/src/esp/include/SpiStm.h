#ifndef __SPISTM_H__
#define __SPISTM_H__

#include "SpiHeader.h"
#include "bsp/ISpiStm.h"
#include "logger/ILogger.h"
#include <BaseTask.h>
#include <Task.h>
#include <array>
#include <driver/spi_slave.h>

class SpiStm : public ISpiStm {
  public:
    SpiStm(ILogger& logger);

    bool send(const uint8_t* buffer, uint16_t length) override;

    bool isBusy() const override;

    void execute();

  private:
    BaseTask<configMINIMAL_STACK_SIZE * 3> m_driverTask;

    ILogger& m_logger;
    enum class transmitState { SENDING_HEADER, SENDING_PAYLOAD, ERROR } m_txState;
    enum class receiveState {
        RECEIVING_HEADER,
        PARSING_HEADER,
        RECEIVING_PAYLOAD,
        VALIDATE_CRC,
        ERROR
    } m_rxState;

    spi_slave_transaction_t m_transaction;

    struct Message {
        // These buffers need to be word-alligned for DMA.
        WORD_ALIGNED_ATTR std::array<uint8_t, STM_SPI_MAX_MESSAGE_LENGTH> m_data;
        uint32_t m_sizeBytes;
    } m_inboundMessage, m_outboundMessage;
    // Used for transaction
    WORD_ALIGNED_ATTR StmHeader::Header m_outboundHeader;
    StmHeader::Header* m_inboundHeader;

    void updateOutboundHeader();
    static void notifyMaster();
    static void transactionCallback(void* context, spi_slave_transaction_t* transaction);

    bool m_isBusy;
};
#endif // __SPISTM_H__
