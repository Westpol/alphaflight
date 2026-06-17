#include "osd.h"
#include "gps.h"
#include "bmp390.h"
#include "power_measurement.h"
#include "crossfire.h"
#include <string.h>

__attribute__((section(".dma_rx"))) static uint8_t osd_dma_buffer[OSD_DMA_BUFFER_SIZE] = {0};
__attribute__((section(".dma_tx"))) static uint8_t osd_response_dma_buffer[64] = {0};

static UART_HandleTypeDef* osd_uart;

static volatile int16_t message_pending = -1;
static volatile uint8_t parser_pos = 0;


OSD_RETURN_TYPE msp_response_packer(const uint8_t* payload, uint8_t type, uint8_t payload_len);

OSD_RETURN_TYPE OSD_INIT(UART_HandleTypeDef* uart){
    if(uart == NULL) return OSD_FAIL;

    osd_uart = uart;

    HAL_UART_Receive_DMA(osd_uart, osd_dma_buffer, OSD_DMA_BUFFER_SIZE);
    __HAL_UART_ENABLE_IT(osd_uart, UART_IT_IDLE);

    return OSD_OKAY;
}

OSD_RETURN_TYPE OSD_IDLE_CALLBACK(void){

    uint8_t current_dma_pointer = OSD_DMA_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(osd_uart->hdmarx);
    uint8_t message_len = parser_pos - current_dma_pointer;     // add message len check

    if(osd_dma_buffer[(uint8_t)(parser_pos)] != '$') goto parser_fail;  // message header check
    if(osd_dma_buffer[(uint8_t)(parser_pos + 1)] != 'M') goto parser_fail;
    if(osd_dma_buffer[(uint8_t)(parser_pos + 2)] != '<') goto parser_fail;
    if(osd_dma_buffer[(uint8_t)(parser_pos + 3)] != '\0') goto parser_fail;

    if(osd_dma_buffer[(uint8_t)(parser_pos + 4)] != osd_dma_buffer[(uint8_t)(parser_pos + 5)]) goto parser_fail; // check if data is correct

    message_pending = osd_dma_buffer[(uint8_t)(parser_pos + 4)];

    parser_pos = current_dma_pointer;

    SCHEDULER_REGISTER_THROWAWAY_TASK(OSD_MSP_RESPONSE, 100, "OSD Response");

    return OSD_OKAY;

    parser_fail:
        parser_pos = current_dma_pointer;
        return OSD_FAIL;
}

uint32_t OSD_MSP_RESPONSE(const task_info_t* task){

    switch (message_pending) {
    case -1:
        return 0;

    case MSP_RAW_GPS:{
        GPS_PROCESSED_T gp = GPS_GET_DATA();
        struct msp_raw_gps_t msp_raw_gps = {MSP_GPS_FIX_3D, gp.status.num_sv, gp.pos.lat, gp.pos.lon, gp.movement.heightMSL / 1000, gp.movement.gspeed / 10, gp.movement.course_over_ground, 1};
        msp_response_packer((uint8_t*)&msp_raw_gps, MSP_RAW_GPS, sizeof(msp_raw_gps));
    }
    break;
    
    case MSP_NAME:{
        char* craft_name = "Highwire Plen";
        msp_response_packer((const uint8_t*)craft_name, MSP_NAME, strlen(craft_name));
    }
    break;

    case MSP_ALTITUDE:{
        BARO_T baro = BARO_GET_DATA_RAW();
        struct msp_altitude_t msp_altitude;

        msp_altitude.estimatedActualPosition = (int32_t)(baro.processed.height * 100);
        msp_altitude.estimatedActualVelocity = (int16_t)baro.processed.vertical_speed;
        msp_altitude.baroLatestAltitude = (int32_t)(baro.processed.height * 100);

        msp_response_packer((uint8_t*)&msp_altitude, MSP_ALTITUDE, sizeof(msp_altitude));
    }
    break;

    case MSP_STATUS:    // implement

    break;

    case MSP_RC:    // implement?

    break;

    case MSP_ATTITUDE:  // implement

    break;

    case MSP_COMP_GPS:  // implement

    break;

    case MSP_BOXIDS:

    break;

    case MSP_ANALOG:    // implement
        struct msp_analog_t msp_analog = {0};
        POWER_DATA_T power_data = POWER_GET_DATA();
        CRSF_LINK_T link_data = CRSF_GET_LINK_DATA();
        msp_analog.vbat = power_data.voltage / 10;   // convert
        msp_analog.mAhDrawn = (uint16_t)power_data.capacity_used_mAh;
        msp_analog.rssi = link_data.down_link_quality;   // not supported by RX yet
        msp_analog.amperage = power_data.current / 10;   // convert
        msp_response_packer((uint8_t*)&msp_analog, MSP_ANALOG, sizeof(msp_analog));
    break;

    case MSP_SET_RTC:

    break;

    default:
    OSD_INIT(NULL);
    break;
    }


    return 0;
}

OSD_RETURN_TYPE msp_response_packer(const uint8_t* payload, uint8_t type, uint8_t payload_len){
    uint8_t frame_length = 0;
    osd_response_dma_buffer[frame_length++] = '$';
    osd_response_dma_buffer[frame_length++] = 'M';
    osd_response_dma_buffer[frame_length++] = '>';
    osd_response_dma_buffer[frame_length++] = payload_len;
    osd_response_dma_buffer[frame_length++] = type;

    memcpy(&osd_response_dma_buffer[frame_length], payload, payload_len);

    frame_length += payload_len;

    uint8_t crc8 = 0;
    crc8 ^= osd_response_dma_buffer[3];
    crc8 ^= osd_response_dma_buffer[4];

    for(int i = 0; i < payload_len; i++){
        crc8 ^= osd_response_dma_buffer[5 + i];
    }

    osd_response_dma_buffer[frame_length++] = crc8;

    HAL_UART_Transmit_DMA(osd_uart, osd_response_dma_buffer, frame_length);

    return OSD_OKAY;
}

UART_HandleTypeDef* OSD_GET_UART(void){
    return osd_uart;
}