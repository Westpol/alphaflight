#include "serial_passthrough.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_dma.h"
#include "stm32h7xx_hal_uart.h"
#include "usbd_cdc_if.h"
#include "usbd_def.h"
#include <string.h>

#define BUFFER_SIZE (STM32_WORD_SIZE * 8)

__attribute__((section(".dma_rx"))) static uint8_t dma_buffer[BUFFER_SIZE] = {0};
__attribute__((section(".dma_tx"))) static uint8_t uart_tx_buffer[BUFFER_SIZE] = {0};

volatile uint8_t pos = 0;
volatile uint8_t old_pos = 0;  // DMA buffer pos marker

volatile UART_HandleTypeDef* local_huart = NULL;
volatile bool passthrough_active = false;

PASSTHROUGH_RETURN_TYPE PASSTHROUGH_START(UART_HandleTypeDef* huart){
    passthrough_active = true;
    local_huart = huart;

    HAL_UART_DMAStop(huart);

    HAL_DMA_Abort(huart->hdmarx);

    HAL_UART_Receive_DMA(huart, dma_buffer, BUFFER_SIZE);  // start UART DMA rx
    
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);  // enable IDLE interrupt, fires when Package is recieved

    while(1){
        if(old_pos != pos){
            uint8_t start_pos = old_pos;
            uint8_t len = pos - old_pos;
            //  send stuff over CDC
            while(CDC_Transmit_HS(&dma_buffer[start_pos], len) == USBD_BUSY){

            }
            old_pos += len;
        }
    }
}

PASSTHROUGH_RETURN_TYPE PASSTHROUGH_USB_RECIEVED(uint8_t* buf, uint32_t len){
    if(len > BUFFER_SIZE) return PASSTHROUGH_FAIL;
    memcpy(uart_tx_buffer, buf, len);
    HAL_UART_Transmit_DMA((UART_HandleTypeDef*)local_huart, uart_tx_buffer, (uint16_t)len);
    return PASSTHROUGH_OKAY;
}

PASSTHROUGH_RETURN_TYPE PASSTHROUGH_UART_CALLBACK(UART_HandleTypeDef* huart){
    if(!passthrough_active) return PASSTHROUGH_FAIL;    // skip ISR if passtrhogh is not activated
    if(huart != local_huart) return PASSTHROUGH_FAIL;   // skip ISR if UART is not correct

    pos = BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart->hdmarx);

    return PASSTHROUGH_OKAY;
}

uint8_t PASSTHROUGH_ACTIVE(void){
    return passthrough_active;
}