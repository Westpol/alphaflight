#include "stm32h723xx.h"
#include "common.h"
#include "stm32h7xx_hal.h"
#include "scheduler.h"

#ifndef GPS_H_
#define GPS_H_

typedef enum{
    GPS_INIT_OKAY,
    GPS_INIT_FAULT,
    GPS_CALLBACK_OKAY,
    GPS_CALLBACK_INTERRUPT_FIRED_MID_TRANSMISSION,
    GPS_FAIL,
    GPS_OKAY
} GPS_RETURN_TYPE;

GPS_RETURN_TYPE GPS_INIT(UART_HandleTypeDef* gps_uart, DMA_HandleTypeDef* gps_uart_dma);
uint32_t GPS_PARSE_DMA(const task_info_t* task);
GPS_RETURN_TYPE GPS_UART_IDLE_CALLBACK();

GPS_RETURN_TYPE GPS_SET_PARSER_TASK_INDEX(int32_t index);

#endif