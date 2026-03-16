#include "spi.h"
#include "stm32h723xx.h"
#include "stm32h7xx_ll_dma.h"
#include "stm32h7xx_ll_gpio.h"
#include "stm32h7xx_ll_spi.h"
#include "timer.h"
#include "main.h"

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
    LL_GPIO_SetOutputPin(cs_port, cs_pin);
    spi_device[device].configured = true;
    return SPI_OKAY;
}

SPI_RETURN_TYPE SPI_ENABLE_DMA(SPI_DEVICE device){
    /*LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_1);    // 2. Enable DMA stream
    LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_0);
    LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_0);    // Enable interrupt before starting
    LL_DMA_EnableIT_TE(DMA2, LL_DMA_STREAM_0); // optional
    LL_DMA_EnableIT_TE(DMA2, LL_DMA_STREAM_1);
    LL_SPI_EnableDMAReq_TX(spi_device[device].spi_peripheral);            // 3. Enable SPI TX requests
    LL_SPI_EnableDMAReq_RX(spi_device[device].spi_peripheral);            // 4. Enable SPI RX requests*/
    return SPI_OKAY;
}

SPI_RETURN_TYPE SPI_START_CS(SPI_DEVICE device){
    LL_GPIO_ResetOutputPin(spi_device[device].cs_port, spi_device[device].cs_pin);
    return SPI_OKAY;
}

SPI_RETURN_TYPE SPI_STOP_CS(SPI_DEVICE device){
    LL_GPIO_ResetOutputPin(spi_device[device].cs_port, spi_device[device].cs_pin);
    return SPI_OKAY;
}

SPI_RETURN_TYPE SPI_TRANSFER_BLOCKING(SPI_DEVICE device, uint8_t* tx_buff, uint8_t* rx_buff, uint8_t len, uint32_t timeout){
    uint32_t start = MICROS32();
    timeout *= 1000;

    uint8_t rx_index = 0;
    uint8_t tx_index = 0;

    if(!spi_device[device].configured) return SPI_NOT_INITIALIZED;

    SPI_TypeDef* spi_peripheral = spi_device[device].spi_peripheral;

        // clear RX FIFO
    while(spi_peripheral->SR & SPI_SR_RXP){     // loop while rx data is in register
        if((MICROS32() - start) >= timeout) goto timeout_cleanup;

        (void)spi_peripheral->RXDR;   // flush rx data
    }

    spi_peripheral->CR2 = len;     // set transfer length

    spi_peripheral->CR1 |= SPI_CR1_SPE;  // enable SPI peripheral

    // pull CS low
    SPI_START_CS(device);

    spi_peripheral->CR1 |= SPI_CR1_CSTART;

    while(rx_index < len || tx_index < len){
        if((MICROS32() - start) >= timeout) goto timeout_cleanup;

        if(spi_peripheral->SR & SPI_SR_TXP && tx_index < len){
            *((__IO uint8_t*)&spi_peripheral->TXDR) = tx_buff[tx_index++];
        }
        if(spi_peripheral->SR & SPI_SR_RXP && rx_index < len){
            rx_buff[rx_index++] = *((__IO uint8_t*)&spi_peripheral->RXDR);
        }
    }

    while(!(spi_peripheral->SR & SPI_SR_EOT)) if((MICROS32() - start) >= timeout) goto timeout_cleanup;     // wait for end of transfer flag

    spi_peripheral->IFCR |= SPI_IFCR_EOTC;  // clear end of transfer flag

    // pull CS high
    SPI_STOP_CS(device);

    spi_peripheral->CR1 &= ~SPI_CR1_SPE;    // disable SPI peripheral

    return SPI_OKAY;

    timeout_cleanup:
        SPI_STOP_CS(device);
        spi_peripheral->CR1 &= ~SPI_CR1_SPE;    // disable SPI peripheral
        return SPI_FAIL;
}

SPI_TypeDef* SPI_GET_DEVICE_PERIPHERAL(SPI_DEVICE device){
    return spi_device[device].spi_peripheral;
}