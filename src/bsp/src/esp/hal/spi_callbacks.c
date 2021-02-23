#include "spi_callbacks.h"

static slaveTransactionCompleteCallback_t callback = NULL;
static void* slaveCallbackInstance = NULL;

void IRAM_ATTR transactionCompleteCallback(spi_slave_transaction_t* transaction) {
    if (slaveCallbackInstance != NULL && callback != NULL) {
        callback(slaveCallbackInstance, transaction);
    }
}

void setCallback(slaveTransactionCompleteCallback_t cb, void* context) {
    callback = cb;
    slaveCallbackInstance = context;
}