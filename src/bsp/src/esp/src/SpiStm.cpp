#include "SpiStm.h"
#include "c-common/software_crc.h"
#include "esp32/rom/crc.h"
#include "hal/pin_map.h"
#include "hal/spi_callbacks.h"
#include <cstring>
#include <driver/gpio.h>

void task(void* instance) {
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

    m_taskHandle = xTaskCreateStatic(task, "esp_spi_driver_task", 4096, this, 1u,
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
        m_logger.log(LogLevel::Debug, "Sending message of length %d", length);
        // Memcpy necessary to have buffer word-alligned for transfer
        std::memcpy(m_outboundMessage.m_data.data(), buffer, length);
        // Padding with 0 up to a word-alligned boundary
        for (uint8_t i = 0; i < (length % 4); i++) {
            m_outboundMessage.m_data[length] = 0;
            length++;
        }
        // Appending CRC32
        *(uint32_t*)(m_outboundMessage.m_data.data() + length) =
            calculateCRC32_software(buffer, length);
        ;
        length += CRC32_SIZE;
        m_outboundMessage.m_sizeBytes = length;
        m_isBusy = true;
        notifyMaster();
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

    switch (m_rxState) {
    case receiveState::RECEIVING_HEADER:
        rxLengthBytes = StmHeader::sizeBytes;
        break;
    case receiveState::PARSING_HEADER:
        m_inboundHeader = (StmHeader::Header*)m_inboundMessage.m_data.data();
        // Validate header
        if (m_inboundHeader->crc8 != calculateCRC8_software(m_inboundHeader, 3)) {
            m_logger.log(LogLevel::Error, "Received corrupted header");
            m_logger.log(LogLevel::Debug, "Bytes were: | %d | %d | %d | %d |",
                         m_inboundMessage.m_data[0], m_inboundMessage.m_data[1],
                         m_inboundMessage.m_data[2], m_inboundMessage.m_data[3]);
            m_rxState = receiveState::ERROR;
            break;
        }

        if (m_inboundHeader->rxSizeWord << 2 == m_outboundMessage.m_sizeBytes &&
            m_outboundMessage.m_sizeBytes != 0) {
            m_logger.log(LogLevel::Debug, "Received valid header. Can now send payload");
            gpio_set_level(STM_USER_0, 0);
            m_txState = transmitState::SENDING_PAYLOAD;
        } else if (m_inboundHeader->rxSizeWord == 0 &&
                   !m_inboundHeader->systemState.espSystemState.failedCrc &&
                   m_outboundMessage.m_sent) {
            m_outboundMessage.m_sizeBytes = 0;
            m_txState = transmitState::SENDING_HEADER;
        } else {
            m_txState = transmitState::SENDING_HEADER;
            m_logger.log(LogLevel::Debug, "Received valid header but cannot send payload");
        }

        // This will be sent on next header. Payload has priority over headers.
        m_inboundMessage.m_sizeBytes = m_inboundHeader->txSizeWord << 2;
        if (m_inboundMessage.m_sizeBytes == m_outboundHeader.rxSizeWord << 2 &&
            m_inboundMessage.m_sizeBytes != 0) {
            rxLengthBytes = m_inboundHeader->txSizeWord << 2;
            m_logger.log(LogLevel::Debug, "Receiving payload");
            m_rxState = receiveState::RECEIVING_PAYLOAD;
        } else {
            rxLengthBytes = StmHeader::sizeBytes;
            m_rxState = receiveState::RECEIVING_HEADER;
        }
        break;
    case receiveState::RECEIVING_PAYLOAD:
        m_outboundHeader.systemState.stmSystemState.failedCrc = 0;
        rxLengthBytes = m_inboundHeader->txSizeWord << 2;
        break;
    case receiveState::VALIDATE_CRC:
        if (calculateCRC32_software(m_inboundMessage.m_data.data(),
                                    m_inboundMessage.m_sizeBytes - CRC32_SIZE) !=
            *(uint32_t*)&m_inboundMessage.m_data[m_inboundMessage.m_sizeBytes - CRC32_SIZE]) {
            m_outboundHeader.systemState.stmSystemState.failedCrc = 1;
        } else {
            m_logger.log(LogLevel::Info, "Stm says: %s", m_inboundMessage.m_data.data());
            m_inboundMessage.m_sizeBytes = 0;
        }
        m_inboundMessage.m_sizeBytes = 0;
        m_rxState = receiveState::RECEIVING_HEADER;
        rxLengthBytes = StmHeader::sizeBytes;
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
        txLengthBytes = StmHeader::sizeBytes;
        break;
    case transmitState::SENDING_PAYLOAD:
        m_outboundMessage.m_sent = true;
        m_transaction.tx_buffer = m_outboundMessage.m_data.data();
        txLengthBytes = m_outboundMessage.m_sizeBytes;
        break;
    case transmitState::ERROR:
        break;
    }

    // The length field of m_transaction is in bits
    m_transaction.length = std::max(rxLengthBytes, txLengthBytes) << 3U;
    // Rx buffer should always be the inbound message buffer.
    m_transaction.rx_buffer = m_inboundMessage.m_data.data();

    if (m_txState != transmitState::ERROR && m_rxState != receiveState::ERROR) {
        // This call is blocking
        spi_slave_transmit(STM_SPI, &m_transaction, portMAX_DELAY);
    }
}

void SpiStm::updateOutboundHeader() {
    // TODO: get actual system state
    m_outboundHeader.txSizeWord = m_outboundMessage.m_sizeBytes >> 2;
    m_outboundHeader.rxSizeWord = m_inboundMessage.m_sizeBytes >> 2;
    m_outboundHeader.crc8 = calculateCRC8_software(&m_outboundHeader, 3);
    if (m_outboundHeader.txSizeWord == 0) {
        m_isBusy = false;
    }
}

void SpiStm::notifyMaster() { gpio_set_level(STM_USER_0, 1); }

void IRAM_ATTR SpiStm::transactionCallback(void* instance, spi_slave_transaction_t* transaction) {
    auto* _this = static_cast<SpiStm*>(instance);
    if (transaction->length != transaction->trans_len) {
        // Transaction timed out before it could finish.
        return;
    }
    switch (_this->m_rxState) {

    case receiveState::RECEIVING_HEADER:
        _this->m_rxState = receiveState::PARSING_HEADER;
        break;
    case receiveState::RECEIVING_PAYLOAD:
        _this->m_rxState = receiveState::VALIDATE_CRC;
        break;
    default:
        // Should never get here
        break;
    }

    if (_this->m_txState == transmitState::SENDING_PAYLOAD) { // TODO: confirm reception with ack
        _this->m_txState = transmitState::SENDING_HEADER;
        _this->m_outboundMessage.m_sizeBytes = 0;
    }
}
