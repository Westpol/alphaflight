
#include "common.h"
#include "stm32h723xx.h"

#ifndef SPI_H_
#define SPI_H_

typedef enum{
    SPI_OKAY,
    SPI_FAIL,
    SPI_TOO_MUCH_DATA
} SPI_RETURN_TYPE;

typedef enum{
    SPI_DEVICE_BARO,
    SPI_DEVICE_IMU,
    SPI_DEVICE_MAGNETO
} SPI_DEVICE;

SPI_RETURN_TYPE SPI_INIT(SPI_DEVICE device, SPI_TypeDef* SPIx, GPIO_TypeDef* cs_port, uint32_t cs_pin);
SPI_RETURN_TYPE SPI_TRANSFER_FIFO(SPI_DEVICE device, uint8_t* tx_buff, uint8_t* rx_buff, uint8_t len);

#endif