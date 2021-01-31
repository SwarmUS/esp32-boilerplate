#include "spi_callbacks.h"

static slaveTransferCompleteCallback slaveTxCompleteCallback = NULL;
static void* slaveCallbackInstance = NULL;

void HAL_SPI_transferCompleteCallback(spi_slave_transaction_t* transaction) {
    if (slaveCallbackInstance != NULL && slaveTxCompleteCallback != NULL) {
        slaveTxCompleteCallback(slaveCallbackInstance, transaction);
    }
}