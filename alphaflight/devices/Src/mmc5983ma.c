#include "mmc5983ma.h"

SPI_DEVICE spi_device = 0;

MAGNETO_RETURN_TYPE MAGNETO_INIT(SPI_DEVICE device){
    spi_device = device;
    const uint8_t tx_buff[2] = {0x80 | 0x2F, 0};
    uint8_t rx_buff[2] = {0};
    SPI_TRANSFER_BLOCKING(SPI_DEVICE_MAGNETO, tx_buff, rx_buff, 2, 10);

    return MAGNETO_OKAY;
}