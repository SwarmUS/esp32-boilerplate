#include "SpiStm.h"
#include "common/CRC.h"
#include "esp32/rom/crc.h"
#include "hal/pin_map.h"
#include "hal/spi_callbacks.h"
#include <cstring>
#include <driver/gpio.h>

void task(void* instance) {
    const uint8_t loopRate = 5;
    while (true) {
        static_cast<SpiStm*>(instance)->execute();
    }
}

SpiStm::SpiStm(ILogger& logger) : m_logger(logger) {
    m_txState = transmitState::SENDING_HEADER;
    m_rxState = receiveState::RECEIVING_HEADER;
    m_inboundMessage = {0};
    m_outboundMessage = {0};
    m_inboundHeader = {0};
    m_outboundHeader = {0};
    m_isBusy = false;

    setCallback(SpiStm::transactionCallback, this);

    m_semaphore = xSemaphoreCreateBinaryStatic(&m_semaphoreBuffer);
    xSemaphoreGive(m_semaphore);

    m_taskHandle = xTaskCreateStatic(task, "esp_spi_driver_task", 1024u, this, 1u,
                                     m_stackData.data(), &m_stackBuffer);
}

bool SpiStm::send(const uint8_t* buffer, uint16_t length) {
    bool retVal = false;

    if (isBusy()) { // Not available
    } else if (length >= STM_SPI_MAX_MESSAGE_LENGTH) { // Message too long
        m_logger.log(LogLevel::Error,
                     "StmEsp: Message length of %d is larger than maximum allowed of %d", length,
                     STM_SPI_MAX_MESSAGE_LENGTH);
    } else if (xSemaphoreTake(m_semaphore, 10) == pdTRUE) {
        m_logger.log(LogLevel::Info, "Sending message of length %d", length);
        // Memcpy necessary to have buffer word-alligned for transfer
        std::memcpy(m_outboundMessage.m_data.data(), buffer, length);
        // Padding with 0 up to a word-alligned boundary
        for (uint8_t  i = 0; i < (length % 4); i++) {
            m_outboundMessage.m_data[length] = 0;
            length++;
        }
        // Appending CRC32
        *(uint32_t*)(m_outboundMessage.m_data.data() + length) =
            CRC::calculateCRC32(buffer, length);
        m_outboundMessage.m_length = length;
        m_isBusy = true;
        xSemaphoreGive(m_semaphore);

        retVal = true;
    } else {
        m_logger.log(LogLevel::Debug, "Failed to acquire semaphore");
    }

    return retVal;
}

bool SpiStm::isBusy() const { return m_isBusy; }

void SpiStm::execute() {
    uint32_t txLengthBytes = 0;
    uint32_t rxLengthBytes = 0;
    uint32_t looprate = 5;
    switch (m_rxState) {
    case receiveState::RECEIVING_HEADER:
        m_transaction.rx_buffer = &m_inboundHeader;
        rxLengthBytes = sizeof(StmSpi::Header);
        //m_logger.log(LogLevel::Error, "Receiving into header buffer");
        break;
    case receiveState::PARSING_HEADER:
        if (m_inboundHeader.headerStruct.crc8 != CRC::calculateCRC8(&m_inboundHeader, 3)) {
            m_logger.log(LogLevel::Error, "Received corrupted payload");
            m_rxState = receiveState::ERROR;
            break;
        } else if (m_inboundHeader.headerStruct.rxSizeWord == m_outboundMessage.m_length) {
            m_txState = transmitState::SENDING_PAYLOAD;
        } else {
            m_txState = transmitState::SENDING_HEADER;
        }
        m_logger.log(LogLevel::Info, "Received valid header");
        break;
    case receiveState::RECEIVING_PAYLOAD:
        m_transaction.rx_buffer = m_inboundMessage.m_data.data();
        rxLengthBytes = sizeof(m_inboundHeader.headerStruct.txSizeWord) << 2;
        m_logger.log(LogLevel::Error, "Receiving into payload buffer");
        break;
    case receiveState::VALID_PAYLOAD:
        m_logger.log(LogLevel::Debug, "Received valid payload");
        m_rxState = receiveState::RECEIVING_HEADER;
        break;
    case receiveState::ERROR:
        m_logger.log(LogLevel::Error, "Error within Spi driver STM");
        m_rxState = receiveState::RECEIVING_HEADER;
        break;
    }

    switch (m_txState) {
    case transmitState::SENDING_HEADER:
        updateOutboundHeader();
        m_transaction.tx_buffer = &m_outboundHeader;
        txLengthBytes = sizeof(StmSpi::Header);
        //m_logger.log(LogLevel::Info, "Output message contains header");
        break;
    case transmitState::SENDING_PAYLOAD:
        m_transaction.tx_buffer = m_outboundMessage.m_data.data();
        txLengthBytes = m_outboundMessage.m_length;
        m_logger.log(LogLevel::Info, "Output message contains payload");
        break;
    case transmitState::ERROR:
        break;
    }

    // The length field of m_transaction is in bits
    m_transaction.length = std::max(rxLengthBytes, txLengthBytes) << 3U;

    // Notify master if payload to send CS is high.
    if (m_outboundMessage.m_length != 0 && gpio_get_level(STM_CS) != 0) {
        notifyMaster();
    }

    // This call is non-blocking.
    spi_slave_queue_trans(STM_SPI, &m_transaction, looprate);
    vTaskDelay(looprate);
}

void SpiStm::updateOutboundHeader() {
    // Reminder: sizes in header are in words, but buffers in bytes
    m_outboundHeader.headerStruct.txSizeWord = m_outboundMessage.m_length >> 2;
    m_outboundHeader.headerStruct.rxSizeWord = m_inboundHeader.headerStruct.txSizeWord >> 2;

    m_outboundHeader.headerStruct.systemState.rawValue = 0; // TODO: get actual system state

    m_outboundHeader.headerStruct.crc8 = CRC::calculateCRC8(&m_outboundHeader, 3);
}

void SpiStm::notifyMaster() { gpio_set_level(STM_USER_0, 1); }

void SpiStm::transactionCallback(void* instance, spi_slave_transaction_t* transaction) {
    auto* _this = static_cast<SpiStm*>(instance);
    if (transaction->length != transaction->trans_len) {
        _this->m_logger.log(LogLevel::Debug, "Transaction timed out before it could finished");
        return;
    }
    switch (_this->m_rxState) {

    case receiveState::RECEIVING_HEADER:
        _this->m_rxState = receiveState::PARSING_HEADER;
        break;
    case receiveState::RECEIVING_PAYLOAD:
        // TODO: validate CRC32
        _this->m_rxState = receiveState::VALID_PAYLOAD;
        break;
    default:
        _this->m_logger.log(LogLevel::Error, "Received data when in unknown state");
        break;
    }

    if (_this->m_txState == transmitState::SENDING_PAYLOAD) { // TODO: confirm reception with ack
        _this->m_txState = transmitState::SENDING_HEADER;
        _this->m_isBusy = false;
    }
}
