#ifndef __SPI_CALLBACKS__
#define __SPI_CALLBACKS__

#ifdef __cplusplus
extern "C" {
#endif

#include <driver/spi_common.h>
#include <driver/spi_slave.h>

typedef void (*slaveTransactionCompleteCallback_t)(void* instance,
                                                   spi_slave_transaction_t* transaction);

/**
 * @brief Function to be set on the configuration as the post transaction callback of the slave
 * interface
 * @param transaction Struct containing the transaction data
 */
void transactionCompleteCallback(spi_slave_transaction_t* transaction);

/**
 * @brief Function to set the callback for completed transaction
 * @param cb Function to be called
 * @param context Context passed to the callback function. Cannot be NULL
 */
void setCallback(slaveTransactionCompleteCallback_t cb, void* context);

#ifdef __cplusplus
}
#endif
#endif // __SPI_CALLBACKS__
