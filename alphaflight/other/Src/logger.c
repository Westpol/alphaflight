#include "logger.h"

#include <string.h>

#include "sd.h"
#include "filesystem.h"
#include "lsm6dso.h"
#include "gps.h"
#include "servo.h"

static log_config_t config = {0};

static uint32_t current_block;  // index of the currently writable (free) block
static uint16_t buff_index = 0; // index of the currently writable (free) byte
static uint32_t flight_num;
static uint8_t log_buff[SD_USABLE_BLOCK_SIZE_BYTES] = {0};
static bool log_active = false;
static uint32_t log_counter = 0;

static int32_t task_index = -1;

static struct __attribute__((packed)){
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
} imu;

static struct __attribute__((packed)){
    uint8_t id;
    uint8_t numSV;
    uint8_t padding[2];
    uint32_t timestamp;
    int32_t lat;
    int32_t lon;
    int32_t speed;
    int32_t height;
    int32_t course;
} gps;

static struct __attribute__((packed)){
    uint8_t id;
    uint8_t padding;
    uint16_t pwm[4];
    uint16_t dshot;
} servo_motor_out_needed;

static void store_timestamp();
static void store_gps();
static void store_imu();
static void store_servo_motor_out();


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
    temp.log_frequency = 50;
    temp.delta_mode = false;

    temp.imu_log_divider = 1;
    temp.gps_log_divider = 10;
    temp.baro_log_divider = 10;
    temp.crsf_log_divider = 2;

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


static void store_imu(){    // ID: 0x03
    IMU_T imu_vals = IMU_GET_RAW_DATA();

    imu.id = 0x03;

    imu.wx = imu_vals.raw.rate_raw.wx;
    imu.wy = imu_vals.raw.rate_raw.wy;
    imu.wz = imu_vals.raw.rate_raw.wz;

    imu.accx = imu_vals.raw.accel_raw.x;
    imu.accy = imu_vals.raw.accel_raw.y;
    imu.accz = imu_vals.raw.accel_raw.z;

    imu.quat_w = (int16_t)UTILS_MIN_MAX_I((int32_t)(imu_vals.processed.quat.w * 10000), INT16_MIN, INT16_MAX);
    imu.quat_x = (int16_t)UTILS_MIN_MAX_I((int32_t)(imu_vals.processed.quat.x * 10000), INT16_MIN, INT16_MAX);
    imu.quat_y = (int16_t)UTILS_MIN_MAX_I((int32_t)(imu_vals.processed.quat.y * 10000), INT16_MIN, INT16_MAX);
    imu.quat_z = (int16_t)UTILS_MIN_MAX_I((int32_t)(imu_vals.processed.quat.z * 10000), INT16_MIN, INT16_MAX);

}

static void store_timestamp(){   // ID: 0x01
    uint32_t now = MILLIS32();
    check_buffer_space(5);
    log_buff[buff_index++] = 0x01;
    memcpy(&log_buff[buff_index], &now, sizeof(now));
    buff_index += 4;
}

static void store_gps(){     // 0x02
    GPS_PROCESSED_T current_vals = GPS_GET_DATA();
    gps.id = 0x02;
    gps.timestamp = current_vals.timestamp;
    gps.lat = current_vals.pos.lat;
    gps.lon = current_vals.pos.lon;
    gps.speed = current_vals.movement.gspeed;
    gps.height = current_vals.movement.heightMSL;
    gps.course = current_vals.movement.course_over_ground;
    gps.numSV = current_vals.status.num_sv;

    check_buffer_space(sizeof(gps));

    memcpy(&log_buff[buff_index], &gps, sizeof(gps));
    buff_index += sizeof(gps);

    return;
}


static void store_servo_motor_out(){        // 0x04
    servo_motor_out_needed.id = 0x04;
    for(uint8_t i = 0; i < 4; i++){
        servo_motor_out_needed.pwm[i] = SERVO_GET_PWM(i);
    }
}