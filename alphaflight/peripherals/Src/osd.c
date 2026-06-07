#include "osd.h"
#include "stm32h7xx_hal_uart.h"

__attribute__((section(".dma_rx"))) static uint8_t osd_dma_buffer[OSD_DMA_BUFFER_SIZE] = {0};

static UART_HandleTypeDef* osd_uart;
static DMA_HandleTypeDef* osd_dma;

OSD_RETURN_TYPE OSD_INIT(UART_HandleTypeDef* uart, DMA_HandleTypeDef* dma){
    if(uart == NULL) return OSD_FAIL;
    if(dma == NULL) return OSD_FAIL;

    osd_uart = uart;
    osd_dma = dma;

    HAL_UART_Receive_DMA(osd_uart, osd_dma_buffer, OSD_DMA_BUFFER_SIZE);

    return OSD_OKAY;
}

