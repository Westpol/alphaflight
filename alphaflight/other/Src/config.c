#include "config.h"
#include "sd.h"
#include <string.h>

static bool config_loaded = false;
static config_entrances_t config_entrances = {0};

static CONFIG_RETURN_TYPE config_set_values_global(void);
static CONFIG_RETURN_TYPE config_get_defaults_global(void);

CONFIG_RETURN_TYPE CONFIG_LOAD_FROM_SD(){

    if(sizeof(config_entrances_t) > SD_USABLE_BLOCK_SIZE_BYTES * CONFIG_SPACE_BLOCKS) return CONFIG_FAIL;   // check if config is bigger than assigned space

    uint8_t buff[SD_USABLE_BLOCK_SIZE_BYTES] = {0};

    for(uint32_t i = 0; i * SD_USABLE_BLOCK_SIZE_BYTES < sizeof(config_entrances_t); i++){
        uint32_t rest = sizeof(config_entrances_t) - i * SD_USABLE_BLOCK_SIZE_BYTES;
        if(SD_READ_BLOCK_BLOCKING(buff, CONFIG_SPACE_START_BLOCK + i, 100) != SD_OKAY) return CONFIG_FAIL;
        memcpy((uint8_t*)&config_entrances + i * SD_USABLE_BLOCK_SIZE_BYTES, &buff[0], UTILS_MIN_I(rest, SD_USABLE_BLOCK_SIZE_BYTES));
    }

    if(config_entrances.version != CONFIG_VERSION) return CONFIG_FAIL;

    config_loaded = true;
    config_set_values_global();
    return CONFIG_OKAY;
}

CONFIG_RETURN_TYPE CONFIG_STORE_TO_SD(){

    config_get_defaults_global();

    if(sizeof(config_entrances_t) > SD_USABLE_BLOCK_SIZE_BYTES * CONFIG_SPACE_BLOCKS) return CONFIG_FAIL;   // check if config is bigger than assigned space

    uint8_t buff[SD_USABLE_BLOCK_SIZE_BYTES] = {0};

    for(uint32_t i = 0; i * SD_USABLE_BLOCK_SIZE_BYTES < sizeof(config_entrances_t); i++){
        uint32_t rest = sizeof(config_entrances_t) - i * SD_USABLE_BLOCK_SIZE_BYTES;
        memcpy(&buff[0], (uint8_t*)&config_entrances + i * SD_USABLE_BLOCK_SIZE_BYTES, UTILS_MIN_I(rest, SD_USABLE_BLOCK_SIZE_BYTES));
        if(SD_WRITE_BLOCK_BLOCKING(buff, CONFIG_SPACE_START_BLOCK + i, 100) != SD_OKAY) return CONFIG_FAIL;
    }

    return CONFIG_OKAY;
}

static CONFIG_RETURN_TYPE config_set_values_global(void){
    if(!config_loaded) return CONFIG_FAIL;
    IMU_SET_CONFIG(config_entrances.imu);
    SERVO_SET_CONFIG(config_entrances.servo);
    return CONFIG_OKAY;
}

static CONFIG_RETURN_TYPE config_get_defaults_global(void){
    config_entrances.imu = IMU_GET_DEFAULT_CONFIG();
    config_entrances.servo = SERVO_GET_DEFAULT_CONFIG();
    config_entrances.version = CONFIG_VERSION;
    return CONFIG_OKAY;
}