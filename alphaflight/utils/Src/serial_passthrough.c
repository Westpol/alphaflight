#include "serial_passthrough.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_dma.h"
#include "usbd_cdc_if.h"
#include "usbd_def.h"
#include <string.h>


// TODO: IMPORTANT! PUT BUFFERS INTO PASSTHROUGH_START FUNCTION AND ONLY CREATE TWO GLOBAL POINTERS, CHECK IN ISR IF POINTER != NULL, OTHERWISE RETURN

__attribute__((section(".dma_rx"))) static uint8_t dma_buffer[STM32_WORD_SIZE * 16] = {0};

static struct{
    uint8_t buff[STM32_WORD_SIZE * 4];   // buffer up to 10 messages if CDC gets stuck (128 bytes each *CHECK AGAINST BIGGEST UBX PACKAGE, INCREASE IF NEEDED*)
    volatile uint16_t tx_len;
} dma_buffer_sec[10];

volatile uint8_t dma_buffer_sec_fill_point = 0; // keep track of num of UBX messages in buffer
volatile uint16_t old_pos = 0;  // DMA buffer pos marker
volatile bool modify_buff = false;

volatile UART_HandleTypeDef* local_huart = NULL;
volatile DMA_HandleTypeDef* local_dma = NULL;
volatile bool passthrough_active = false;

PASSTHROUGH_RETURN_TYPE PASSTHROUGH_START(UART_HandleTypeDef* huart, DMA_HandleTypeDef* hdma_rx){
    passthrough_active = true;
    local_huart = huart;
    local_dma = hdma_rx;

    HAL_UART_Receive_DMA(huart, dma_buffer, STM32_WORD_SIZE * 16);  // start UART DMA rx
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);  // enable IDLE interrupt, fires when Package is recieved

    while(1){
        if(dma_buffer_sec_fill_point > 0){  // if buff isn't empty, try to start transaction
            modify_buff = true;
            uint16_t idx = dma_buffer_sec_fill_point - 1;
            if(CDC_Transmit_HS(dma_buffer_sec[idx].buff, dma_buffer_sec[idx].tx_len) == USBD_OK) dma_buffer_sec_fill_point--;
            modify_buff = false;
        }
    }
}

PASSTHROUGH_RETURN_TYPE PASSTHROUGH_UART3_CALLBACK(){
    if(!passthrough_active) return PASSTHROUGH_FAIL;    // skip ISR if passtrhogh is not activated

    if (__HAL_UART_GET_FLAG(local_huart, UART_FLAG_IDLE)){
        __HAL_UART_CLEAR_IDLEFLAG(local_huart); // clear idle flag

        uint16_t dma_buff_pos = STM32_WORD_SIZE * 16 - __HAL_DMA_GET_COUNTER(local_dma);    // calc buffer pos

        if(dma_buffer_sec_fill_point >= 10){
            old_pos = dma_buff_pos;
            return PASSTHROUGH_FAIL;
        }
        if(modify_buff){
            old_pos = dma_buff_pos;
            return PASSTHROUGH_FAIL;
        }
        if(old_pos > dma_buff_pos){
            old_pos = dma_buff_pos;
            return PASSTHROUGH_FAIL;
        }

        memcpy(dma_buffer_sec[dma_buffer_sec_fill_point++].buff, &dma_buffer[old_pos], dma_buff_pos - old_pos);  // copy packet into next buffer
        dma_buffer_sec[dma_buffer_sec_fill_point - 1].tx_len = dma_buff_pos - old_pos;
        old_pos = dma_buff_pos;
    }
    return PASSTHROUGH_OKAY;
}