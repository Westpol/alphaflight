#include "serial_passthrough.h"
#include "stm32h7xx_hal.h"
#include "usbd_cdc_if.h"

__attribute__((section(".dma_rx"))) static uint8_t dma_buffer[STM32_WORD_SIZE * 16] = {0};

PASSTHROUGH_RETURN_TYPE PASSTHROUGH_START(UART_HandleTypeDef* huart){
    HAL_UART_Receive_DMA(huart, dma_buffer, 512);
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);

    while(1){
        CDC_Transmit_HS(uint8_t *Buf, uint16_t Len);
    }
}

PASSTHROUGH_RETURN_TYPE PASSTHROUGH_UART2_CALLBACK(){

    return PASSTHROUGH_OKAY;
}