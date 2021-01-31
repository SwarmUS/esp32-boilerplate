#ifndef __SPI_CALLBACKS__
#define __SPI_CALLBACKS__

#ifdef __cplusplus
extern "C" {
#endif

#include <driver/spi_common.h>
#include <driver/spi_slave.h>

typedef void (*slaveTransferCompleteCallback)(void* instance, spi_slave_transaction_t* transaction);

void HAL_SPI_transferCompleteCallback(spi_slave_transaction_t* transaction);

#ifdef __cplusplus
}
#endif
#endif // __SPI_CALLBACKS__
