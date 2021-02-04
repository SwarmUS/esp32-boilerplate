#include "spi_callbacks.h"

static slaveTransactionCompleteCallback callback = NULL;
static void* slaveCallbackInstance = NULL;

void transactionCompleteCallback(spi_slave_transaction_t* transaction) {
    if (slaveCallbackInstance != NULL && callback != NULL) {
        callback(slaveCallbackInstance, transaction);
    }
}

void setCallback(slaveTransactionCompleteCallback cb, void* instance) {
    callback = cb;
    slaveCallbackInstance = instance;
}