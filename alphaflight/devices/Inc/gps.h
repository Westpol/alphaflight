#include "stm32h723xx.h"
#include "common.h"
#include "stm32h7xx_hal.h"
#include "scheduler.h"
#include <stdint.h>

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

typedef struct{
    int32_t lat;
    int32_t lon;
} gps_pos_t;

typedef struct{
    int32_t gspeed;
    int32_t course_over_ground;
    int32_t heightMSL;
    int32_t velN;
    int32_t velE;
    int32_t velD;
} gps_movement_t;

typedef struct{
    uint8_t fix_type;
    uint8_t num_sv;
} gps_status_t;

typedef struct{
    uint32_t horizontal_acc;
    uint32_t speed_acc;
    uint32_t vertical_acc;
    uint32_t heading_acc;
} gps_accuracy_t;

typedef struct{
    uint32_t timestamp;
    gps_pos_t pos;
    gps_movement_t movement;
    gps_status_t status;
    gps_accuracy_t acc;
} GPS_PROCESSED_T;

GPS_RETURN_TYPE GPS_INIT(UART_HandleTypeDef* gps_uart, DMA_HandleTypeDef* gps_uart_dma);
uint32_t GPS_PARSE_DMA(const task_info_t* task);
GPS_RETURN_TYPE GPS_UART_IDLE_CALLBACK();

GPS_PROCESSED_T GPS_GET_DATA();

GPS_RETURN_TYPE GPS_SET_PARSER_TASK_INDEX(int32_t index);

#endif