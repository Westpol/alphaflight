
#include "common.h"
#include "stm32h723xx.h"
#include "stm32h7xx_hal.h"
#include <stdint.h>

#ifndef SPI_H_
#define SPI_H_

typedef enum{
    SPI_OKAY,
    SPI_FAIL,
    SPI_TOO_MUCH_DATA,
    SPI_NOT_INITIALIZED
} SPI_RETURN_TYPE;

typedef enum{
    SPI_DEVICE_BARO,
    SPI_DEVICE_IMU,
    SPI_DEVICE_MAGNETO
} SPI_DEVICE;

SPI_RETURN_TYPE SPI_INIT(SPI_DEVICE device, SPI_HandleTypeDef* SPIx, GPIO_TypeDef* cs_port, uint32_t cs_pin);
SPI_RETURN_TYPE SPI_TRANSFER_DMA(SPI_DEVICE device, const uint8_t* tx_buff, uint8_t* rx_buff, uint8_t len);
SPI_RETURN_TYPE SPI_TRANSFER_BLOCKING(SPI_DEVICE device, const uint8_t* tx_buff, uint8_t* rx_buff, uint8_t len, uint32_t timeout);
SPI_RETURN_TYPE SPI_START_CS(SPI_DEVICE device);
SPI_RETURN_TYPE SPI_STOP_CS(SPI_DEVICE device);

SPI_HandleTypeDef* SPI_GET_DEVICE_PERIPHERAL(SPI_DEVICE device);

#endif