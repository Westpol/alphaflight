#include "logger.h"
#include "sd.h"
#include "filesystem.h"
#include "lsm6dso.h"
#include <stdint.h>
#include <string.h>

static log_config_t config = {0};

static uint32_t current_block;  // index of the currently writable (free) block
static uint16_t buff_index = 0; // index of the currently writable (free) byte
static uint32_t flight_num;
static uint8_t log_buff[SD_USABLE_BLOCK_SIZE_BYTES] = {0};
static bool log_active = false;
static uint32_t log_counter = 0;

static int32_t task_index = -1;

static IMU_PROCESSED_T last_imu_vals;

static uint32_t last_millis = 0;

static void store_timestamp(void);


LOG_RETURN_TYPE LOG_START(){
    current_block = FS_NEW_FLIGHT();
    flight_num = FS_GET_FN();
    log_active = true;
    SCHEDULER_ENABLE_TASK_BY_INDEX(task_index);
    return LOG_OKAY;
}

uint32_t LOG_RUN(const task_info_t* task){
    if(!log_active) return 0;
    store_timestamp();
    log_counter++;
    return 0;
}

LOG_RETURN_TYPE LOG_END(){
    log_active = false;
    FS_END_FLIGHT(current_block);
    SCHEDULER_DISABLE_TASK_BY_INDEX(task_index);
    return LOG_OKAY;
}

void LOG_SET_CONFIG(log_config_t new_config){
    config = new_config;
}

void LOG_SET_TASK_INDEX(int32_t index){
    task_index = index;
}

log_config_t LOG_GET_DEFAULT_CONFIG(){
    log_config_t temp = {0};
    temp.log_frequency = 100;
    temp.delta_mode = true;

    temp.imu_log_divider = 1;
    temp.gps_log_divider = 100;
    temp.baro_log_divider = 50;
    temp.crsf_log_divider = 5;

    return temp;
}

uint32_t LOG_GET_FREQUENCY(void){
    return config.log_frequency;
}

static void store_imu(void){    // ID: 0x03

}

static void check_buffer_space(uint8_t space_checked){
    if(buff_index + space_checked > SD_USABLE_BLOCK_SIZE_BYTES){
        memset(&log_buff[buff_index], 0, SD_USABLE_BLOCK_SIZE_BYTES - buff_index);
        SD_WRITE_BLOCK_DMA(log_buff, current_block++);
        memcpy(log_buff, &flight_num, sizeof(uint32_t));
        buff_index = 4;
    }
}


static void store_timestamp(void){
    uint32_t now = MILLIS32();
    uint32_t delta = now - last_millis;
    if(delta > 0xFF){  // absolute package   // ID: 0x01
        check_buffer_space(5);
        log_buff[buff_index++] = 0x01;
        memcpy(&log_buff[buff_index], &now, sizeof(now));
        buff_index += 4;
    }
    else {  //relative package   // ID: 0x02
        check_buffer_space(2);
        log_buff[buff_index++] = 0x02;
        log_buff[buff_index++] = (uint8_t)delta;
    }
    last_millis = now;
}