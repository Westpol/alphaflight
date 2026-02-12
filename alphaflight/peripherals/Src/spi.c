#include "spi.h"
#include "stm32h723xx.h"
#include "stm32h7xx_ll_gpio.h"
#include "stm32h7xx_ll_spi.h"

#define num_devices 3   // baro, IMU, Magneto

static struct{
    SPI_TypeDef* spi_peripheral;
    GPIO_TypeDef* cs_port;
    uint32_t cs_pin;
    bool configured;
} spi_device[num_devices];

SPI_RETURN_TYPE SPI_INIT(SPI_DEVICE device, SPI_TypeDef* SPIx, GPIO_TypeDef* cs_port, uint32_t cs_pin){
    spi_device[device].spi_peripheral = SPIx;
    spi_device[device].cs_port = cs_port;
    spi_device[device].cs_pin = cs_pin;
    LL_SPI_Enable(SPIx);
    LL_GPIO_SetOutputPin(cs_port, cs_pin);
    spi_device[device].configured = true;
    return SPI_OKAY;
}

SPI_RETURN_TYPE SPI_TRANSFER_FIFO(SPI_DEVICE device, uint8_t* tx_buff, uint8_t* rx_buff, uint8_t len){
    if(len > 16) return SPI_TOO_MUCH_DATA;
    if(!spi_device[device].configured) return SPI_NOT_INITIALIZED;

    SPI_TypeDef* spi_peripheral = spi_device[device].spi_peripheral;

    // clear RX FIFO
    while(LL_SPI_IsActiveFlag_RXP(spi_peripheral)){
        (void)LL_SPI_ReceiveData8(spi_peripheral);
    }

    LL_SPI_SetTransferSize(spi_peripheral, len);

    // load fifo with tx data
    for(uint8_t i = 0; i < len; i++){
        LL_SPI_TransmitData8(spi_peripheral, tx_buff[i]);
    }

    // pull CS low
    LL_GPIO_ResetOutputPin(spi_device[device].cs_port, spi_device[device].cs_pin);

    LL_SPI_StartMasterTransfer(spi_peripheral);

    // wait until message has been sent
    while(!LL_SPI_IsActiveFlag_EOT(spi_peripheral));
    LL_SPI_ClearFlag_EOT(spi_peripheral);

    // flush fifo into rx buffer
    for(uint8_t i = 0; i < len; i++){
        while(!LL_SPI_IsActiveFlag_RXP(spi_peripheral));
        rx_buff[i] = LL_SPI_ReceiveData8(spi_peripheral);
    }

    // pull CS high
    LL_GPIO_SetOutputPin(spi_device[device].cs_port, spi_device[device].cs_pin);

    return SPI_OKAY;
}

SPI_TypeDef* SPI_GET_DEVICE_PERIPHERAL(SPI_DEVICE device){
    return spi_device[device].spi_peripheral;
}