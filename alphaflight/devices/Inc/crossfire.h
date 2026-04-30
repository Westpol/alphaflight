#include "common.h"
#include "stm32h7xx_hal.h"
#include "scheduler.h"

#ifndef CROSSFIRE_H
#define CROSSFIRE_H

typedef enum{
    CRSF_FAIL,
    CRSF_OKAY
} CRSF_RETURN_TYPE;

typedef struct{
    uint8_t dummy;
} crsf_config_t;

typedef struct{
    uint16_t channel[16];
    uint32_t timestamp;
} CRSF_CHANNELS_T;

typedef struct{
    uint8_t     up_rssi_ant1;       // Uplink RSSI Antenna 1 (dBm * -1)
    uint8_t     up_rssi_ant2;       // Uplink RSSI Antenna 2 (dBm * -1)
    uint8_t     up_link_quality;    // Uplink Package success rate / Link quality (%)
    int8_t      up_snr;             // Uplink SNR (dB)
    uint8_t     active_antenna;     // number of currently best antenna
    uint8_t     rf_profile;         // enum {4fps = 0 , 50fps, 150fps}
    uint8_t     up_rf_power;        // enum {0mW = 0, 10mW, 25mW, 100mW,
                                    // 500mW, 1000mW, 2000mW, 250mW, 50mW}
    uint8_t     down_rssi;          // Downlink RSSI (dBm * -1)
    uint8_t     down_link_quality;  // Downlink Package success rate / Link quality (%)
    int8_t      down_snr;           // Downlink SNR (dB)
    uint32_t timestamp;
} CRSF_LINK_T;

CRSF_RETURN_TYPE CRSF_INIT(UART_HandleTypeDef* uart, DMA_HandleTypeDef* crsf_uart_dma);
uint32_t CRSF_PARSE_DMA(const task_info_t* task);
uint32_t CRSF_TELEMETRY(const task_info_t* task);
CRSF_RETURN_TYPE CRSF_UART_IDLE_CALLBACK();

CRSF_CHANNELS_T CRSF_GET_CHANNELS(void);
CRSF_LINK_T CRSF_GET_LINK(void);

CRSF_RETURN_TYPE CRSF_SET_PARSER_TASK_PID(int32_t index);

#endif