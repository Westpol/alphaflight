#include "gps.h"
#include "stm32h7xx_ll_usart.h"
#include "timer.h"

#define recieve_to_parse_time_offset 1000       // in microseconds
#define timing_filter_value 9

static USART_TypeDef* gps_uart;

static volatile struct{
    uint32_t package_recieved_timestamp;        // package timestamps
    uint32_t last_package_recieved_timestamp;
    bool new_package;
    uint32_t package_skipped;

    uint32_t average_package_delta;
    uint32_t next_execution_delta;
} gps_task_timing;

uint8_t buffer_point;

__attribute__((section(".dma_rx"))) static uint8_t dma_buffer[255] = {0};

static bool validate_ubx_crc(uint8_t* buffer, uint8_t len, uint8_t checksum_a, uint8_t checksum_b){
    uint8_t ck_a = 0, ck_b = 0;

    for(uint8_t i = 0; i < len; i++){
        ck_a = ck_a + buffer[i];
        ck_b = ck_b + ck_a;
    }

    return (ck_a == checksum_a && ck_b == checksum_b);
}

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

uint32_t GPS_PARSE_BUFFER(){

    if(!gps_task_timing.new_package) return 2000UL;      // try again in 2 ms

    // TODO: add parsing logic

    validate_ubx_crc(&dma_buffer[10], 120, 0, 0);

    gps_task_timing.new_package = false;

    uint32_t package_delta = gps_task_timing.package_recieved_timestamp - gps_task_timing.last_package_recieved_timestamp;
    gps_task_timing.last_package_recieved_timestamp = gps_task_timing.package_recieved_timestamp;       // set last timestamp to now for next delta calculation when new package has arrived

    gps_task_timing.average_package_delta = (timing_filter_value * gps_task_timing.average_package_delta + package_delta) / (timing_filter_value + 1);      // simple lowpass filter to avoid jittering
    gps_task_timing.next_execution_delta = gps_task_timing.package_recieved_timestamp + gps_task_timing.average_package_delta + recieve_to_parse_time_offset;

    return gps_task_timing.next_execution_delta;
}

GPS_RETURN_TYPE GPS_UART_IDLE_CALLBACK(){
    uint32_t now = MICROS32();

    if(now - gps_task_timing.package_recieved_timestamp < 10000UL){     // do not count as package loss if less than 10ms has passed since last callback (accidental idle in the middle of a message probably)
        gps_task_timing.package_recieved_timestamp = now;
        gps_task_timing.new_package = true;
        return GPS_CALLBACK_OKAY;
    }

    gps_task_timing.package_recieved_timestamp = now;        // timestamp new package
    if(gps_task_timing.new_package) gps_task_timing.package_skipped++;      // check if last package was parsed
    gps_task_timing.new_package = true;     // mark arrival of new package

    return GPS_CALLBACK_OKAY;
}