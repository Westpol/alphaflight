#include "logger.h"
#include "sd.h"
#include "filesystem.h"
#include "lsm6dso.h"
#include "gps.h"
#include <string.h>

static log_config_t config = {0};

static uint32_t current_block;  // index of the currently writable (free) block
static uint16_t buff_index = 0; // index of the currently writable (free) byte
static uint32_t flight_num;
static uint8_t log_buff[SD_USABLE_BLOCK_SIZE_BYTES] = {0};
static bool log_active = false;
static uint32_t log_counter = 0;

static int32_t task_index = -1;

static IMU_T imu_vals;

static struct{
    uint8_t id;
    uint8_t padding;
    int16_t wx;
    int16_t wy;
    int16_t wz;

    int16_t accx;
    int16_t accy;
    int16_t accz;

    int16_t quat_w;
    int16_t quat_x;
    int16_t quat_y;
    int16_t quat_z;

    int16_t pitch;
    int16_t roll;
}imu_absolute, imu_last;

static struct{
    uint8_t id;
    int8_t wx;
    int8_t wy;
    int8_t wz;

    int8_t accx;
    int8_t accy;
    int8_t accz;

    int8_t quat_w;
    int8_t quat_x;
    int8_t quat_y;
    int8_t quat_z;

    int8_t pitch;
    int8_t roll;
}imu_delta;

static struct{
    uint8_t id;
    int8_t numSV;
    uint32_t timestamp;
    int32_t lat;
    int32_t lon;
    int32_t speed;
    int32_t height;
    int32_t course;
} gps_absolute, gps_absolute_last;

static struct{
    uint8_t id;
    int8_t numSV;
    uint8_t timestamp;
    int16_t lat;
    int16_t lon;
    int16_t speed;
    int16_t height;
    int16_t course;
} gps_delta;

static inline bool check_oob_int16(int32_t delta){return delta <= INT16_MAX && delta >= INT16_MIN;}

static inline bool check_oob_unit16(int32_t delta){return delta <= UINT16_MAX && delta >= 0;}

static inline bool check_oob_int8(int32_t delta){return delta <= INT8_MAX && delta >= INT8_MIN;}

static inline bool check_oob_uint8(int32_t delta){return delta <= UINT8_MAX && delta >= 0;}

static uint32_t last_millis = 0;

static void store_timestamp(bool force_absolute);
static void store_gps(bool force_absolute);
static void store_imu(bool force_absolute);


LOG_RETURN_TYPE LOG_START(){
    current_block = FS_NEW_FLIGHT();
    flight_num = FS_GET_FN();

    // set flight num at first log block
    memcpy(log_buff, &flight_num, sizeof(flight_num));
    buff_index = 4;

    store_timestamp(true);
    store_gps(true);

    log_active = true;
    SCHEDULER_ENABLE_TASK_BY_INDEX(task_index);
    return LOG_OKAY;
}

uint32_t LOG_RUN(const task_info_t* task){
    if(!log_active) return 0;

    store_timestamp(false);
    store_imu(false);

    if(log_counter >= 100){
        store_gps(false);
        log_counter = 0;
    }

    log_counter++;
    return 0;
}

LOG_RETURN_TYPE LOG_END(){
    log_active = false;
    FS_END_FLIGHT(current_block);
    // write last block
    memset(&log_buff[buff_index], 0, SD_USABLE_BLOCK_SIZE_BYTES - buff_index);
    SD_WRITE_BLOCK_DMA(log_buff, current_block);
    SCHEDULER_DISABLE_TASK_BY_INDEX(task_index);
    return LOG_OKAY;
}


void LOG_SET_TASK_PID(int32_t index){
    task_index = index;
}


void LOG_SET_CONFIG(log_config_t new_config){
    config = new_config;
}


log_config_t LOG_GET_CONFIG(void){
    return config;
}


log_config_t LOG_GET_DEFAULT_CONFIG(void){
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

static void check_buffer_space(uint8_t space_checked){
    if(buff_index + space_checked > SD_USABLE_BLOCK_SIZE_BYTES){
        memset(&log_buff[buff_index], 0, SD_USABLE_BLOCK_SIZE_BYTES - buff_index);
        SD_WRITE_BLOCK_DMA(log_buff, current_block++);
        memcpy(log_buff, &flight_num, sizeof(uint32_t));
        buff_index = 4;
    }
}


static void store_imu(bool force_absolute){    // ID: 0x05 or 0x06
    imu_vals = IMU_GET_RAW_DATA();

    imu_absolute.wx = imu_vals.raw.rate_raw.wx;
    imu_absolute.wy = imu_vals.raw.rate_raw.wy;
    imu_absolute.wz = imu_vals.raw.rate_raw.wz;

    imu_absolute.accx = imu_vals.raw.accel_raw.x;
    imu_absolute.accy = imu_vals.raw.accel_raw.y;
    imu_absolute.accz = imu_vals.raw.accel_raw.z;

    imu_absolute.quat_w = (int16_t)UTILS_MIN_MAX_I((int32_t)(imu_vals.processed.quat.w * 10000), INT16_MIN, INT16_MAX);
    imu_absolute.quat_x = (int16_t)UTILS_MIN_MAX_I((int32_t)(imu_vals.processed.quat.x * 10000), INT16_MIN, INT16_MAX);
    imu_absolute.quat_y = (int16_t)UTILS_MIN_MAX_I((int32_t)(imu_vals.processed.quat.y * 10000), INT16_MIN, INT16_MAX);
    imu_absolute.quat_z = (int16_t)UTILS_MIN_MAX_I((int32_t)(imu_vals.processed.quat.z * 10000), INT16_MIN, INT16_MAX);

}

static void store_timestamp(bool force_absolute){
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

static void store_gps(bool force_absolute){     // 0x03 or 0x04
    GPS_PROCESSED_T current_vals = GPS_GET_DATA();
    gps_absolute.id = 0x03;
    gps_absolute.timestamp = current_vals.timestamp;
    gps_absolute.lat = current_vals.pos.lat;
    gps_absolute.lon = current_vals.pos.lon;
    gps_absolute.speed = current_vals.movement.gspeed;
    gps_absolute.height = current_vals.movement.heightMSL;
    gps_absolute.course = current_vals.movement.course_over_ground;
    gps_absolute.numSV = current_vals.status.num_sv;

    if(force_absolute) goto absolute;

    if(gps_absolute.timestamp - gps_absolute_last.timestamp > 0xFF) goto absolute;

    int32_t delta = gps_absolute.lat - gps_absolute_last.lat;
    if(check_oob_int16(delta)) goto absolute;
    gps_delta.lat = (int16_t) delta;
    
    delta = gps_absolute.lon - gps_absolute_last.lon;
    if(check_oob_int16(delta)) goto absolute;
    gps_delta.lon = (int16_t) delta;

    delta = gps_absolute.speed - gps_absolute_last.speed;
    if(check_oob_int16(delta)) goto absolute;
    gps_delta.speed = (int16_t) delta;

    delta = gps_absolute.height - gps_absolute_last.height;
    if(check_oob_int16(delta)) goto absolute;
    gps_delta.height = (int16_t) delta;

    delta = gps_absolute.course - gps_absolute_last.course;
    if(check_oob_int16(delta)) goto absolute;
    gps_delta.course = (int16_t) delta;

    gps_delta.numSV = gps_absolute.numSV - gps_absolute_last.numSV;

    gps_delta.id = 0x04;

    check_buffer_space(sizeof(gps_delta));

    memcpy(&log_buff[buff_index], &gps_delta, sizeof(gps_delta));
    buff_index += sizeof(gps_delta);

    memcpy(&gps_absolute_last, &gps_absolute, sizeof(gps_absolute));

    return;

    absolute:
        check_buffer_space(sizeof(gps_absolute));

        memcpy(&log_buff[buff_index], &gps_absolute, sizeof(gps_absolute));
        buff_index += sizeof(gps_absolute);

        memcpy(&gps_absolute_last, &gps_absolute, sizeof(gps_absolute));

        return;
}