#include "stm32h723xx.h"

#ifndef GPS_H_
#define GPS_H_

typedef enum{
    GPS_INIT_OKAY,
    GPS_INIT_FAULT,
    GPS_CALLBACK_OKAY
} GPS_RETURN_TYPE;

GPS_RETURN_TYPE GPS_INIT(USART_TypeDef* gps_uart, uint8_t update_rate);
GPS_RETURN_TYPE GPS_UART_IDLE_CALLBACK();

#endif