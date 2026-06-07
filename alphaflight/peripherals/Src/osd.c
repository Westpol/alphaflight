#include "osd.h"
#include "gps.h"
#include <string.h>

__attribute__((section(".dma_rx"))) static uint8_t osd_dma_buffer[OSD_DMA_BUFFER_SIZE] = {0};
__attribute__((section(".dma_tx"))) static uint8_t osd_response_dma_buffer[64] = {0};

static UART_HandleTypeDef* osd_uart;
static DMA_HandleTypeDef* osd_dma;

static volatile int16_t message_pending = -1;
static volatile uint8_t parser_pos = 0;


OSD_RETURN_TYPE OSD_INIT(UART_HandleTypeDef* uart, DMA_HandleTypeDef* dma){
    if(uart == NULL) return OSD_FAIL;
    if(dma == NULL) return OSD_FAIL;

    osd_uart = uart;
    osd_dma = dma;

    HAL_UART_Receive_DMA(osd_uart, osd_dma_buffer, OSD_DMA_BUFFER_SIZE);
    __HAL_UART_ENABLE_IT(osd_uart, UART_IT_IDLE);

    return OSD_OKAY;
}

OSD_RETURN_TYPE OSD_IDLE_CALLBACK(void){

    uint8_t current_dma_pointer = OSD_DMA_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(osd_dma);
    uint8_t message_len = parser_pos - current_dma_pointer;

    if(osd_dma_buffer[(uint8_t)(parser_pos)] != '$') goto parser_fail;  // message header check
    if(osd_dma_buffer[(uint8_t)(parser_pos + 1)] != 'M') goto parser_fail;
    if(osd_dma_buffer[(uint8_t)(parser_pos + 2)] != '<') goto parser_fail;
    if(osd_dma_buffer[(uint8_t)(parser_pos + 3)] != '\0') goto parser_fail;

    if(osd_dma_buffer[(uint8_t)(parser_pos + 4)] != osd_dma_buffer[(uint8_t)(parser_pos + 5)]) goto parser_fail; // check if data is correct

    message_pending = osd_dma_buffer[(uint8_t)(parser_pos + 4)];

    parser_pos = current_dma_pointer;

    if(message_pending == MSP_RAW_GPS){
        SCHEDULER_REGISTER_THROWAWAY_TASK(OSD_MSP_RESPONSE, 100, "OSD Response");
    }

    return OSD_OKAY;

    parser_fail:
        parser_pos = current_dma_pointer;
        return OSD_FAIL;
}

uint32_t OSD_MSP_RESPONSE(const task_info_t* task){
    //HAL_UART_Transmit_DMA(osd_uart, const uint8_t *pData, uint16_t Size)

        // MSP_RAW_GPS reply
    struct msp_raw_gps_t {
    uint8_t  fixType;       // MSP_GPS_NO_FIX, MSP_GPS_FIX_2D, MSP_GPS_FIX_3D
    uint8_t  numSat;
    int32_t  lat;           // 1 / 10000000 deg
    int32_t  lon;           // 1 / 10000000 deg
    int16_t  alt;           // meters
    int16_t  groundSpeed;   // cm/s
    int16_t  groundCourse;  // unit: degree x 10
    uint16_t hdop;
    } __attribute__ ((packed));

    GPS_PROCESSED_T gp = GPS_GET_DATA();

    struct msp_raw_gps_t msp_raw_gps = {MSP_GPS_FIX_3D, gp.status.num_sv, gp.pos.lat, gp.pos.lon, gp.movement.heightMSL, gp.movement.gspeed, gp.movement.course_over_ground, 1};

    memcpy(osd_response_dma_buffer, &msp_raw_gps, sizeof(msp_raw_gps));

    HAL_UART_Transmit_DMA(osd_uart, osd_response_dma_buffer, sizeof(msp_raw_gps));

    return 0;
}