#include "common.h"
#include "stm32h7xx_hal.h"
#include "scheduler.h"

#ifndef CROSSFIRE_H
#define CROSSFIRE_H

typedef enum{
    CRSF_FAIL,
    CRSF_OKAY
} CRSF_RETURN_TYPE;

CRSF_RETURN_TYPE CRSF_INIT(UART_HandleTypeDef* uart, DMA_HandleTypeDef* crsf_uart_dma);
uint32_t CRSF_PARSE_DMA(const task_info_t* task);
CRSF_RETURN_TYPE CRSF_UART_IDLE_CALLBACK();

CRSF_RETURN_TYPE CRSF_SET_PARSER_TASK_INDEX(int32_t index);

#endif