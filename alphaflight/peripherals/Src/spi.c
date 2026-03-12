#include "spi.h"
#include "stm32h723xx.h"
#include "stm32h7xx_ll_gpio.h"

#define num_devices 3   // baro, IMU, Magneto

static struct{
    SPI_HandleTypeDef* spi_peripheral;
    GPIO_TypeDef* cs_port;
    uint32_t cs_pin;
    bool configured;
} spi_device[num_devices];

SPI_RETURN_TYPE SPI_INIT(SPI_DEVICE device, SPI_HandleTypeDef* SPIx, GPIO_TypeDef* cs_port, uint32_t cs_pin){
    spi_device[device].spi_peripheral = SPIx;
    spi_device[device].cs_port = cs_port;
    spi_device[device].cs_pin = cs_pin;
    LL_GPIO_SetOutputPin(cs_port, cs_pin);
    spi_device[device].configured = true;
    return SPI_OKAY;
}

SPI_RETURN_TYPE SPI_START_CS(SPI_DEVICE device){
    LL_GPIO_ResetOutputPin(spi_device[device].cs_port, spi_device[device].cs_pin);
    return SPI_OKAY;
}

SPI_RETURN_TYPE SPI_STOP_CS(SPI_DEVICE device){
    LL_GPIO_SetOutputPin(spi_device[device].cs_port, spi_device[device].cs_pin);
    return SPI_OKAY;
}

SPI_RETURN_TYPE SPI_TRANSFER_DMA(SPI_DEVICE device, const uint8_t* tx_buff, uint8_t* rx_buff, uint8_t len){
    if(!spi_device[device].configured) return SPI_NOT_INITIALIZED;
    SPI_START_CS(device);
    HAL_SPI_TransmitReceive_DMA(spi_device[device].spi_peripheral, &tx_buff[0], &rx_buff[0], len);
    return SPI_OKAY;
}

SPI_RETURN_TYPE SPI_TRANSFER(SPI_DEVICE device, const uint8_t* tx_buff, uint8_t* rx_buff, uint8_t len){
    if(len > 16) return SPI_TOO_MUCH_DATA;
    if(!spi_device[device].configured) return SPI_NOT_INITIALIZED;

    // pull CS low
    SPI_START_CS(device);

    HAL_SPI_TransmitReceive(spi_device[device].spi_peripheral, &tx_buff[0], &rx_buff[0], len, 100);

    // pull CS high
    SPI_STOP_CS(device);

    return SPI_OKAY;
}

SPI_HandleTypeDef* SPI_GET_DEVICE_PERIPHERAL(SPI_DEVICE device){
    return spi_device[device].spi_peripheral;
}