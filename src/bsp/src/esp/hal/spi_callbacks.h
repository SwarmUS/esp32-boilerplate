#ifndef __SPI_CALLBACKS__
#define __SPI_CALLBACKS__

#ifdef __cplusplus
extern "C" {
#endif

#include <driver/spi_common.h>
#include <driver/spi_slave.h>

typedef void (*slaveTransactionCompleteCallback_t)(void* instance,
                                                 spi_slave_transaction_t* transaction);

void transactionCompleteCallback(spi_slave_transaction_t* transaction);

void setCallback(slaveTransactionCompleteCallback_t cb, void* context);

#ifdef __cplusplus
}
#endif
#endif // __SPI_CALLBACKS__
