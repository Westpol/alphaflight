#include "logger.h"
#include "sd.h"
#include "filesystem.h"

static log_config_t config = {0};

static uint32_t current_block;
static uint16_t buff_pos = 0;
static uint32_t flight_num;
static uint8_t log_buff[SD_USABLE_BLOCK_SIZE_BYTES] = {0};
static bool log_active = false;

LOG_RETURN_TYPE LOG_START(){
    current_block = FS_NEW_FLIGHT();
    flight_num = FS_GET_FN();
    log_active = true;
    return LOG_OKAY;
}

uint32_t LOG_RUN(const task_info_t* task){
    if(!log_active) return 0;
}

LOG_RETURN_TYPE LOG_END(){
    log_active = false;
    FS_END_FLIGHT(current_block);
    return LOG_OKAY;
}

void LOG_SET_CONFIG(log_config_t new_config){
    config = new_config;
}

log_config_t LOG_GET_DEFAULT_CONFIG(){
    log_config_t temp = {0};
    temp.log_frequency = 1000;
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