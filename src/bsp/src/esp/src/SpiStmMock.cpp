#include "SpiStm.h"
#include "c-common/software_crc.h"
#include "esp32/rom/crc.h"
#include "hal/pin_map.h"
#include "hal/spi_callbacks.h"
#include <cstring>
#include <driver/gpio.h>

/** These macros are used to convert the units of the size of a buffer from a number of words to a
 * number of bytes and conversely from a size in bytes to a number of words, and similarly between a
 * sizes in bits and bytes.
 */
#define WORDS_TO_BYTES(words) ((words << 2U))
#define BYTES_TO_WORDS(bytes) ((bytes >> 2U))
#define BYTES_TO_BITS(bytes) ((bytes << 3U))

void task(void* context) {
    while (true) {
        static_cast<SpiStm*>(context)->execute();
    }
}

SpiStm::SpiStm(ILogger& logger) :
    m_logger(logger), m_driverTask("stm_spi_driver", tskIDLE_PRIORITY + 1, task, this) {
    m_txState = transmitState::SENDING_HEADER;
    m_rxState = receiveState::RECEIVING_HEADER;
    m_inboundMessage = {0};
    m_outboundMessage = {0};
    m_inboundHeader = {0};
    m_outboundHeader = {0};
    m_isBusy = false;
    m_isConnected = false;

    CircularBuff_init(&m_circularBuf, m_data.data(), m_data.size());

    setCallback(SpiStm::transactionCallback, this);

    m_driverTask.start();
    // Notifying master for first handshake
    notifyMaster();
}

bool SpiStm::receive(uint8_t* data, uint16_t length) {
    if (data == nullptr || length > STM_SPI_MAX_MESSAGE_LENGTH) {
        m_logger.log(LogLevel::Warn, "Invalid parameters for SpiStm::Receive");
        return false;
    }
    m_receivingTaskHandle = xTaskGetCurrentTaskHandle();
    while (CircularBuff_getLength(&m_circularBuf) < length) {
        ulTaskNotifyTake(pdTRUE, 500);
        // TODO: check for disconnection
    }
    m_receivingTaskHandle = nullptr;

    return CircularBuff_get(&m_circularBuf, data, length) == length;
}

bool SpiStm::send(const uint8_t* buffer, uint16_t length) {
    if (length >= STM_SPI_MAX_MESSAGE_LENGTH) { // Message too long
        m_logger.log(LogLevel::Error,
                     "StmEsp: Message length of %d is larger than maximum allowed of %d", length,
                     STM_SPI_MAX_MESSAGE_LENGTH);
        return false;
    }

    m_logger.log(LogLevel::Debug, "Sending message of length %d", length);
    // Memcpy necessary to have buffer word-alligned for transfer
    std::memcpy(m_outboundMessage.m_data.data(), buffer, length);
    // Set payload size in header
    m_outboundHeader.payloadSize = length;
    // Padding with 0 up to a word-alligned boundary
    for (uint8_t i = 0; i < (length % 4); i++) {
        m_outboundMessage.m_data[length] = 0;
        length++;
    }
    // Appending CRC32
    *(uint32_t*)(m_outboundMessage.m_data.data() + length) =
        calculateCRC32_software(buffer, length);
    length += CRC32_SIZE;
    m_outboundMessage.m_sizeBytes = length;
    m_isBusy = true;
    notifyMaster();

    // Wait for transmission to be over
    m_sendingTaskHandle = xTaskGetCurrentTaskHandle();
    while (isBusy()) {
        ulTaskNotifyTake(pdTRUE, 500);
        if (m_hasSentPayload && m_inboundHeader->systemState.stmSystemState.failedCrc) {
            // Crc failed, handle retries in the future
            m_sendingTaskHandle = nullptr;
            return false;
        }
        // TODO: check for disconnection
    }
    m_sendingTaskHandle = nullptr;
    return true;
}

bool SpiStm::isBusy() const { return m_isBusy; }

bool SpiStm::isConnected() const {
    if (!m_isConnected) {
        notifyMaster();
    }
    return m_isConnected;
}

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
            m_logger.log(LogLevel::Error, "Received corrupted STM SPI header");
            m_logger.log(LogLevel::Debug, "Bytes were: | %d | %d | %d | %d |",
                         m_inboundMessage.m_data[0], m_inboundMessage.m_data[1],
                         m_inboundMessage.m_data[2], m_inboundMessage.m_data[3]);
            m_rxState = receiveState::ERROR;
            m_isConnected = false;
            break;
        }
        m_isConnected = true;
        if (WORDS_TO_BYTES(m_inboundHeader->rxSizeWord) == m_outboundMessage.m_sizeBytes &&
            m_outboundMessage.m_sizeBytes != 0) {
            m_logger.log(LogLevel::Debug, "Received valid header. Can now send payload");
            // Reset gpio trigger once header has been received
            gpio_set_level(STM_USER_0, 0);
            m_txState = transmitState::SENDING_PAYLOAD;
        } else {
            m_txState = transmitState::SENDING_HEADER;
            m_logger.log(LogLevel::Debug, "Received valid header but cannot send payload");
        }

        // This will be sent on next header. Payload has priority over headers.
        m_inboundMessage.m_sizeBytes = WORDS_TO_BYTES(m_inboundHeader->txSizeWord);
        if (m_inboundMessage.m_sizeBytes == WORDS_TO_BYTES(m_outboundHeader.rxSizeWord) &&
            m_inboundMessage.m_sizeBytes != 0) {
            rxLengthBytes = WORDS_TO_BYTES(m_inboundHeader->txSizeWord);
            m_logger.log(LogLevel::Debug, "Receiving payload");
            m_rxState = receiveState::RECEIVING_PAYLOAD;
        } else {
            rxLengthBytes = StmHeader::sizeBytes;
            m_rxState = receiveState::RECEIVING_HEADER;
        }
        break;
    case receiveState::RECEIVING_PAYLOAD:
        rxLengthBytes = WORDS_TO_BYTES(m_inboundHeader->txSizeWord);
        break;
    case receiveState::VALIDATE_CRC:
        // Check payload CRC and log an error and set flag if it fails
        if (calculateCRC32_software(m_inboundMessage.m_data.data(),
                                    m_inboundMessage.m_sizeBytes - CRC32_SIZE) !=
            *(uint32_t*)&m_inboundMessage.m_data[m_inboundMessage.m_sizeBytes - CRC32_SIZE]) {
            m_logger.log(LogLevel::Error, "Failed payload crc on STM");
            m_outboundHeader.systemState.stmSystemState.failedCrc = 1;
        }
        // If it passes the CRC check, add the data to the circular buffer
        else if (CircularBuff_put(&m_circularBuf, m_inboundMessage.m_data.data(),
                                  m_inboundMessage.m_sizeBytes - CRC32_SIZE) ==
                 CircularBuff_Ret_Ok) {
            // If a task was waiting to receive bytes, notify it
            if (m_receivingTaskHandle != nullptr) {
                xTaskNotifyGive(m_receivingTaskHandle);
            }
        } else {
            m_logger.log(LogLevel::Error, "Failed to add bytes in spi circular buffer");
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
        m_transaction.tx_buffer = m_outboundMessage.m_data.data();
        txLengthBytes = m_outboundMessage.m_sizeBytes;
        m_hasSentPayload = false;
        break;
    case transmitState::ERROR:
        m_logger.log(LogLevel::Error, "Error within Spi driver STM");
        break;
    }

    // The length field of m_transaction is in bits
    m_transaction.length = BYTES_TO_BITS(std::max(rxLengthBytes, txLengthBytes));
    // Rx buffer should always be the inbound message buffer.
    m_transaction.rx_buffer = m_inboundMessage.m_data.data();

    if (m_txState != transmitState::ERROR && m_rxState != receiveState::ERROR) {
        // This call is blocking, so the rate of the of the loop is inferred byt the rate of the
        // loop of the master driver in HiveMind. The loop needs no delay and shouldn't
        // have one as it will only increase latency, which could lead to instability.
        spi_slave_transmit(STM_SPI, &m_transaction, portMAX_DELAY);
    }
}

void SpiStm::updateOutboundHeader() {
    // TODO: get actual system state
    m_outboundHeader.txSizeWord = BYTES_TO_WORDS(m_outboundMessage.m_sizeBytes);
    m_outboundHeader.rxSizeWord = BYTES_TO_WORDS(m_inboundMessage.m_sizeBytes);
    m_outboundHeader.padding = 0;
    m_outboundHeader.crc8 = calculateCRC8_software(&m_outboundHeader, 3);
    if (m_outboundHeader.txSizeWord == 0) {
        m_isBusy = false;
    }
}

void SpiStm::notifyMaster() {
    gpio_set_level(STM_USER_0, 0);
    Task::delay(1);
    gpio_set_level(STM_USER_0, 1);
}

void IRAM_ATTR SpiStm::transactionCallback(void* context, spi_slave_transaction_t* transaction) {
    auto* instance = static_cast<SpiStm*>(context);
    if (transaction->length != transaction->trans_len) {
        // Transaction timed out before it could finish.
        return;
    }
    switch (instance->m_rxState) {

    case receiveState::RECEIVING_HEADER:
        instance->m_rxState = receiveState::PARSING_HEADER;
        break;
    case receiveState::RECEIVING_PAYLOAD:
        instance->m_rxState = receiveState::VALIDATE_CRC;
        break;
    default:
        // This should never be called. The state machine should never be in any other state during
        // the ISR.
        instance->m_logger.log(LogLevel::Error, "Interrupted called on invalid state");
        instance->m_txState = transmitState::ERROR;
        break;
    }

    if (instance->m_txState == transmitState::SENDING_PAYLOAD) { // TODO: confirm reception with ack
        instance->m_txState = transmitState::SENDING_HEADER;
        instance->m_outboundMessage.m_sizeBytes = 0;
        instance->m_hasSentPayload = true;
        instance->m_isBusy = false;
        // notify sending task
        if (instance->m_sendingTaskHandle != nullptr) {
            // Note: esp does not need the flag for yielding from ISR
            vTaskNotifyGiveFromISR(instance->m_sendingTaskHandle, nullptr);
            portYIELD_FROM_ISR();
        }
    }
}
