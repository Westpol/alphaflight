#include "gps.h"
#include "timer.h"
#include <time.h>

static UART_HandleTypeDef* gps_uart;
static DMA_HandleTypeDef* gps_dma;

static volatile int32_t gps_parser_task_index = -1;

static volatile struct{
    uint16_t write_point;
    uint16_t read_point;
} gps_parser = {0};

static bool new_uart_data_arrived = false;
//mktime(struct tm *_timeptr)
typedef struct __attribute__((packed)){
    uint32_t iTOW;
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
    uint8_t valid;
    uint32_t tAcc;
    int32_t nano;
    uint8_t fixType;
    uint8_t flags;
    uint8_t flags2;
    uint8_t numSV;
    int32_t lon;    // 1e-7 deg
    int32_t lat;    // 1e-7 deg
    int32_t height; // mm
    int32_t hMSL;   // mm
    uint32_t hAcc;  // mm
    uint32_t vAcc;  // mm
    int32_t velN;   // mm/s
    int32_t velE;   // mm/s
    int32_t velD;   // mm/s
    int32_t gSpeed; // mm/s
    int32_t headMot; // 1e-5 deg
    uint32_t sAcc;  // mm/s
    uint32_t headAcc; // 1e-5 deg
    uint16_t pDOP;  // 0.01
    uint16_t flags3;
    uint8_t reserved[4];
    int32_t headVeh; // 1e-5 deg
    int16_t magDec; // 1e-2 deg
    uint16_t magAcc; // 1e-2 deg
} gps_nav_pvt_t;

static struct __attribute__((packed)){
    uint8_t header_1;
    uint8_t header_2;
    uint8_t class;
    uint8_t id;
    uint16_t len;
    gps_nav_pvt_t data;
    uint8_t checksum_a;
    uint8_t checksum_b;
} gps_nav_pvt;

#define DMA_BUFFER_SIZE STM32_WORD_SIZE * 16
__attribute__((section(".dma_rx"))) static uint8_t gps_dma_buffer[DMA_BUFFER_SIZE] = {0};

static GPS_PROCESSED_T gps_processed = {0};

static bool validate_ubx_crc(uint8_t* buffer, uint16_t len, uint8_t checksum_a, uint8_t checksum_b){
    uint8_t ck_a = 0, ck_b = 0;

    for(uint8_t i = 0; i < len; i++){
        ck_a = ck_a + buffer[i];
        ck_b = ck_b + ck_a;
    }

    return (ck_a == checksum_a && ck_b == checksum_b);
}

static uint32_t GPS_DATETIME_TO_UNIX(uint16_t year, uint8_t month, uint8_t day,
                                     uint8_t hour, uint8_t min, uint8_t sec) {
    if (year < 2025) return 0;

    // Days per month, non-leap year
    static const uint8_t days_in_month[] = { 31,28,31,30,31,30,31,31,30,31,30,31 };

    uint32_t days = 0;

    // Add days for years since 1970
    for (uint16_t y = 1970; y < year; y++) {
        days += (y % 4 == 0 && (y % 100 != 0 || y % 400 == 0)) ? 366 : 365;
    }

    // Add days for months in current year
    for (uint8_t m = 1; m < month; m++) {
        days += days_in_month[m - 1];
        if (m == 2 && (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0))) {
            days += 1; // Leap day
        }
    }

    // Add days in current month
    days += day - 1;

    return days * 86400 + hour * 3600 + min * 60 + sec;
}

static GPS_RETURN_TYPE gps_convert_data(void){

    gps_processed.pos.lat = gps_nav_pvt.data.lat;
    gps_processed.pos.lon = gps_nav_pvt.data.lon;

    gps_processed.movement.course_over_ground = gps_nav_pvt.data.headMot;
    gps_processed.movement.gspeed = gps_nav_pvt.data.gSpeed;
    gps_processed.movement.heightMSL = gps_nav_pvt.data.hMSL;
    gps_processed.movement.velD = gps_nav_pvt.data.velD;
    gps_processed.movement.velE = gps_nav_pvt.data.velE;
    gps_processed.movement.velN = gps_nav_pvt.data.velN;

    gps_processed.status.fix_type = gps_nav_pvt.data.fixType;
    gps_processed.status.num_sv = gps_nav_pvt.data.numSV;

    gps_processed.acc.heading_acc = gps_nav_pvt.data.headAcc;
    gps_processed.acc.horizontal_acc = gps_nav_pvt.data.hAcc;
    gps_processed.acc.speed_acc = gps_nav_pvt.data.sAcc;
    gps_processed.acc.vertical_acc = gps_nav_pvt.data.vAcc;

    gps_processed.timestamp = GPS_DATETIME_TO_UNIX(gps_nav_pvt.data.year, gps_nav_pvt.data.month, gps_nav_pvt.data.day, gps_nav_pvt.data.hour, gps_nav_pvt.data.min, gps_nav_pvt.data.sec);

    return GPS_OKAY;
}

GPS_RETURN_TYPE GPS_INIT(UART_HandleTypeDef* uart, DMA_HandleTypeDef* gps_uart_dma){   // Assumes UART already initialized with correct settings
    if(uart == NULL) return GPS_INIT_FAULT;
    if(gps_uart_dma == NULL) return GPS_INIT_FAULT;
    gps_dma = gps_uart_dma;
    gps_uart = uart;

    HAL_UART_Receive_DMA(gps_uart, gps_dma_buffer, DMA_BUFFER_SIZE);
    __HAL_UART_ENABLE_IT(gps_uart, UART_IT_IDLE);

    return GPS_INIT_OKAY;
}


uint32_t GPS_PARSE_DMA(const task_info_t* task){
    new_uart_data_arrived = false;

    uint16_t read_so_far = gps_parser.read_point;
    uint16_t write_so_far = gps_parser.write_point;

    if(read_so_far < write_so_far){ // no buffer wrap
        for(uint16_t i = read_so_far; i < write_so_far - 5; i++){   // search new data for header
            if(0xb5 == gps_dma_buffer[i] && 0x62 == gps_dma_buffer[i + 1]){ // check for header
                uint8_t class = gps_dma_buffer[i + 2];
                uint8_t id = gps_dma_buffer[i + 3];
                if(class == 0x01 && id == 0x07){    // nav pvt
                    uint16_t len = ((uint16_t)gps_dma_buffer[i + 5] << 8) | gps_dma_buffer[i + 4];
                    if(i + len + 8 > write_so_far) return 1;    // not all of the packet there yet, try again later

                    if(len != sizeof(gps_nav_pvt_t)) continue;  // lengths don't match, start parsing from new position
                    // crc doesn't match, start parsing from new position
                    if(!validate_ubx_crc(&gps_dma_buffer[i + 2], len + 4, gps_dma_buffer[i + len + 6], gps_dma_buffer[i + len + 7])) continue;

                    for(uint16_t f = 0; f < sizeof(gps_nav_pvt); f++){  // copy message bytewise over to padded struct
                        *((uint8_t*)&gps_nav_pvt + f) = gps_dma_buffer[i + f];
                    }

                    gps_parser.read_point = i + len + 8;
                    gps_convert_data();
                    goto found_packet;
                }
                else if(class == 0x01 && id == 0x12){   // velned
                    uint16_t len = ((uint16_t)gps_dma_buffer[i + 5] << 8) | gps_dma_buffer[i + 4];
                    if(i + len + 8 > write_so_far) return 1;    // not all of the packet there yet, try again later
                    gps_parser.read_point = i + len + 8;
                    goto found_packet;
                }
            }
        }
        gps_parser.read_point = UTILS_MAX_I(write_so_far - 7, 0);
    }
    else{
        gps_parser.read_point = write_so_far;   // skip wrapped around package
    }

    found_packet:
        if(!new_uart_data_arrived) SCHEDULER_DISABLE_TASK_BY_INDEX(gps_parser_task_index);  // only deactivate if no new interrupt fired while parsing
        return 0;
}


GPS_RETURN_TYPE GPS_UART_IDLE_CALLBACK(){
    if(gps_parser_task_index == -1) return GPS_FAIL;
    new_uart_data_arrived = true;
    gps_parser.write_point = DMA_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(gps_dma);
    SCHEDULER_ENABLE_TASK_BY_INDEX(gps_parser_task_index);
    return GPS_CALLBACK_OKAY;
}

GPS_RETURN_TYPE GPS_SET_PARSER_TASK_INDEX(int32_t index){
    gps_parser_task_index = index;
    SCHEDULER_DISABLE_TASK_BY_INDEX(gps_parser_task_index);
    return GPS_OKAY;
}