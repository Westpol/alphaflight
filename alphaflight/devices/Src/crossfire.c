#include "crossfire.h"
#include "stm32h723xx.h"
#include "timer.h"
#include <stdint.h>

static UART_HandleTypeDef* crsf_uart;
static DMA_HandleTypeDef* crsf_dma;

static volatile int32_t crsf_parser_task_index = -1;

static volatile struct{
    uint16_t write_point;
    uint16_t read_point;
} crsf_parser = {0};

static bool new_uart_data_arrived = false;

#define DMA_BUFFER_SIZE STM32_WORD_SIZE * 4
__attribute__((section(".dma_rx"))) static uint8_t crsf_dma_buffer[DMA_BUFFER_SIZE] = {0};
static uint8_t crsf_packet[64] = {0};

static struct{
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
}crsf_fc_link_statistics;

static struct{
    uint16_t channel[16];
}crsf_fc_channels;

static uint8_t crc8(const uint8_t * ptr, uint8_t len);


CRSF_RETURN_TYPE CRSF_INIT(UART_HandleTypeDef* uart, DMA_HandleTypeDef* crsf_uart_dma){   // Assumes UART already initialized with correct settings
    if(uart == NULL) return CRSF_FAIL;
    if(crsf_uart_dma == NULL) return CRSF_FAIL;
    crsf_dma = crsf_uart_dma;
    crsf_uart = uart;

    HAL_UART_Receive_DMA(crsf_uart, crsf_dma_buffer, DMA_BUFFER_SIZE);
    __HAL_DMA_ENABLE_IT(crsf_dma, DMA_IT_HT);
    __HAL_DMA_ENABLE_IT(crsf_dma, DMA_IT_TC);

    return CRSF_OKAY;
}


uint32_t CRSF_PARSE_DMA(const task_info_t* task){
    new_uart_data_arrived = false;

    uint16_t write_so_far = crsf_parser.write_point;

    for(uint16_t i = crsf_parser.read_point; i + 1 < write_so_far; i++){
        if(crsf_dma_buffer[i] == 0xC8){ // packets for FC (link stats and channels)
            uint8_t len = crsf_dma_buffer[i + 1];
            if(len < 2 || len > 62) continue;   // invalid len, probably picked up stray header

            if(i + 2 + len < DMA_BUFFER_SIZE){  // no wrap around
                if(i + 2 + len > write_so_far){    // if not all data recieved yet, stay at parsing point and wait for complete packet
                    crsf_parser.read_point = i;
                    break;
                }
                for(uint8_t f = 0; f < len; f++){   // all data recieved
                    crsf_packet[f] = crsf_dma_buffer[f + i + 2];    // load packet in buffer starting from type (needed for CRC)
                }
                uint32_t crc = crc8(crsf_packet, len - 1);
                if(crc != crsf_packet[len - 1]) continue;   // wrong CRC, skip packet
                uint8_t type = crsf_packet[0];
                if(type == 0x16){   // channels
                    // crsf_packet[0] = Type (0x16)
                    // Channel data starts at crsf_packet[1]

                    crsf_fc_channels.channel[0]  = (crsf_packet[1]       | crsf_packet[2] << 8) & 0x07FF;
                    crsf_fc_channels.channel[1]  = (crsf_packet[2] >> 3  | crsf_packet[3] << 5) & 0x07FF;
                    crsf_fc_channels.channel[2]  = (crsf_packet[3] >> 6  | crsf_packet[4] << 2 | crsf_packet[5] << 10) & 0x07FF;
                    crsf_fc_channels.channel[3]  = (crsf_packet[5] >> 1  | crsf_packet[6] << 7) & 0x07FF;
                    crsf_fc_channels.channel[4]  = (crsf_packet[6] >> 4  | crsf_packet[7] << 4) & 0x07FF;
                    crsf_fc_channels.channel[5]  = (crsf_packet[7] >> 7  | crsf_packet[8] << 1 | crsf_packet[9] << 9) & 0x07FF;
                    crsf_fc_channels.channel[6]  = (crsf_packet[9] >> 2  | crsf_packet[10] << 6) & 0x07FF;
                    crsf_fc_channels.channel[7]  = (crsf_packet[10] >> 5 | crsf_packet[11] << 3) & 0x07FF;

                    crsf_fc_channels.channel[8]  = (crsf_packet[12]      | crsf_packet[13] << 8) & 0x07FF;
                    crsf_fc_channels.channel[9]  = (crsf_packet[13] >> 3 | crsf_packet[14] << 5) & 0x07FF;
                    crsf_fc_channels.channel[10] = (crsf_packet[14] >> 6 | crsf_packet[15] << 2 | crsf_packet[16] << 10) & 0x07FF;
                    crsf_fc_channels.channel[11] = (crsf_packet[16] >> 1 | crsf_packet[17] << 7) & 0x07FF;
                    crsf_fc_channels.channel[12] = (crsf_packet[17] >> 4 | crsf_packet[18] << 4) & 0x07FF;
                    crsf_fc_channels.channel[13] = (crsf_packet[18] >> 7 | crsf_packet[19] << 1 | crsf_packet[20] << 9) & 0x07FF;
                    crsf_fc_channels.channel[14] = (crsf_packet[20] >> 2 | crsf_packet[21] << 6) & 0x07FF;
                    crsf_fc_channels.channel[15] = (crsf_packet[21] >> 5 | crsf_packet[22] << 3) & 0x07FF;
                }
                if(type == 0x14){   // link stats
                    for(uint8_t f = 0; f < len - 2; f++){
                        *((uint8_t*)&crsf_fc_link_statistics + f) = crsf_packet[f + 1];
                    }
                }
                i += len + 2 - 1;   // move to next packet, subtract one because of for loop increment
            }
            else{
                // handle buffer wrap
            }
        }
    }
    crsf_parser.read_point = write_so_far;
    SCHEDULER_DISABLE_TASK_BY_INDEX(crsf_parser_task_index);
    return 0;
}


CRSF_RETURN_TYPE CRSF_DMA_CALLBACK(){
    if(crsf_parser_task_index == -1) return CRSF_FAIL;
    crsf_parser.write_point = DMA_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(crsf_dma);
    SCHEDULER_ENABLE_TASK_BY_INDEX(crsf_parser_task_index);
    return CRSF_OKAY;
}

CRSF_RETURN_TYPE CRSF_SET_PARSER_TASK_INDEX(int32_t index){
    crsf_parser_task_index = index;
    return CRSF_OKAY;
}

static uint8_t crc8tab[256] = {
    0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54, 0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
    0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06, 0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
    0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0, 0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
    0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2, 0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
    0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9, 0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
    0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B, 0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
    0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D, 0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
    0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F, 0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
    0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB, 0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
    0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9, 0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
    0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F, 0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
    0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D, 0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
    0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26, 0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
    0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74, 0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
    0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82, 0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
    0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0, 0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9
};

uint8_t crc8(const uint8_t * ptr, uint8_t len){
    uint8_t crc = 0;
    for (uint8_t i=0; i<len; i++)
        crc = crc8tab[crc ^ *ptr++];
    return crc;
}