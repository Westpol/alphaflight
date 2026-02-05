#include "gps.h"
#include "stm32h7xx_ll_usart.h"
#include <stdint.h>
#include <stdbool.h>
#include "timer.h"

static USART_TypeDef* gps_uart;

static volatile struct{
    uint32_t package_recieved_timestamp;
    bool new_package;
    uint32_t package_skipped;
} gps_task_timing;

GPS_RETURN_TYPE GPS_INIT(USART_TypeDef *uart, uint8_t update_rate){   // Assumes UART already initialized with correct settings
    if(uart == NULL) return GPS_INIT_FAULT;

    gps_uart = uart;

    LL_USART_DisableIT_RXNE(uart);   // RX data
    LL_USART_DisableIT_TXE(uart);    // TX empty
    LL_USART_DisableIT_TC(uart);     // TX complete
    LL_USART_DisableIT_PE(uart);     // parity error
    LL_USART_DisableIT_ERROR(uart);  // FE, NE, ORE

    LL_USART_EnableIT_IDLE(uart);    // only IDLE

    // TODO: change GPS settings in blocking mode to desired specs

    return GPS_INIT_OKAY;
}

GPS_RETURN_TYPE GPS_UART_IDLE_CALLBACK(){

    gps_task_timing.package_recieved_timestamp = MICROS32();        // timestamp new package
    if(gps_task_timing.new_package) gps_task_timing.package_skipped++;      // check if last package was parsed
    gps_task_timing.new_package = true;     // mark arrival of new package

    return GPS_CALLBACK_OKAY;
}