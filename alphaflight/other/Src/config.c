#include "config.h"
#include "sd.h"
#include <string.h>

static bool config_loaded = false;
static config_entrances_t config_entrances = {0};

static CONFIG_RETURN_TYPE config_set_global(void);
static CONFIG_RETURN_TYPE config_get_defaults_global(void);
static CONFIG_RETURN_TYPE config_get_global(void);

CONFIG_RETURN_TYPE CONFIG_LOAD_FROM_SD(){

    if(sizeof(config_entrances_t) > SD_USABLE_BLOCK_SIZE_BYTES * CONFIG_SPACE_BLOCKS) return CONFIG_FAIL;   // check if config is bigger than assigned space

    uint8_t buff[SD_USABLE_BLOCK_SIZE_BYTES] = {0};

    for(uint32_t i = 0; i * SD_USABLE_BLOCK_SIZE_BYTES < sizeof(config_entrances_t); i++){
        uint32_t rest = sizeof(config_entrances_t) - i * SD_USABLE_BLOCK_SIZE_BYTES;
        if(SD_READ_BLOCK_BLOCKING(buff, CONFIG_SPACE_START_BLOCK + i, sizeof(buff), 100) != SD_OKAY) return CONFIG_FAIL;
        memcpy((uint8_t*)&config_entrances + i * SD_USABLE_BLOCK_SIZE_BYTES, &buff[0], UTILS_MIN_UI(rest, SD_USABLE_BLOCK_SIZE_BYTES));
    }

    if(config_entrances.version != CONFIG_VERSION) return CONFIG_FAIL;

    config_loaded = true;
    config_set_global();
    return CONFIG_OKAY;
}

CONFIG_RETURN_TYPE CONFIG_STORE_TO_SD(){

    config_get_global();

    if(sizeof(config_entrances_t) > SD_USABLE_BLOCK_SIZE_BYTES * CONFIG_SPACE_BLOCKS) return CONFIG_FAIL;   // check if config is bigger than assigned space

    uint8_t buff[SD_USABLE_BLOCK_SIZE_BYTES] = {0};

    for(uint32_t i = 0; i * SD_USABLE_BLOCK_SIZE_BYTES < sizeof(config_entrances_t); i++){
        uint32_t rest = sizeof(config_entrances_t) - i * SD_USABLE_BLOCK_SIZE_BYTES;
        memcpy(&buff[0], (uint8_t*)&config_entrances + i * SD_USABLE_BLOCK_SIZE_BYTES, UTILS_MIN_UI(rest, SD_USABLE_BLOCK_SIZE_BYTES));
        if(SD_WRITE_BLOCK_BLOCKING(buff, CONFIG_SPACE_START_BLOCK + i, sizeof(buff), 100) != SD_OKAY) return CONFIG_FAIL;
    }

    return CONFIG_OKAY;
}

CONFIG_RETURN_TYPE CONFIG_STORE_TO_SD_DEFAULTS(){

    config_get_defaults_global();

    if(sizeof(config_entrances_t) > SD_USABLE_BLOCK_SIZE_BYTES * CONFIG_SPACE_BLOCKS) return CONFIG_FAIL;   // check if config is bigger than assigned space

    uint8_t buff[SD_USABLE_BLOCK_SIZE_BYTES] = {0};

    for(uint32_t i = 0; i * SD_USABLE_BLOCK_SIZE_BYTES < sizeof(config_entrances_t); i++){
        uint32_t rest = sizeof(config_entrances_t) - i * SD_USABLE_BLOCK_SIZE_BYTES;
        memcpy(&buff[0], (uint8_t*)&config_entrances + i * SD_USABLE_BLOCK_SIZE_BYTES, UTILS_MIN_UI(rest, SD_USABLE_BLOCK_SIZE_BYTES));
        if(SD_WRITE_BLOCK_BLOCKING(buff, CONFIG_SPACE_START_BLOCK + i, sizeof(buff), 100) != SD_OKAY) return CONFIG_FAIL;
    }

    return CONFIG_OKAY;
}

CONFIG_RETURN_TYPE CONFIG_LOAD_DEFAULTS(void){
    config_get_defaults_global();
    return CONFIG_OKAY;
}


#define CONFIG_MODULE_LIST(X) \
    X(IMU, imu)              \
    X(SERVO, servo)          \
    X(DSHOT, dshot)          \
    X(LOG, logger)           \
    X(POWER_MEASUREMENT, power_measurement)

static CONFIG_RETURN_TYPE config_set_global(void){
    if(!config_loaded) return CONFIG_FAIL;
    #define APPLY_SET(module, field) module##_SET_CONFIG(config_entrances.field);
    CONFIG_MODULE_LIST(APPLY_SET)
    #undef APPLY_SET
    return CONFIG_OKAY;
}


static CONFIG_RETURN_TYPE config_get_global(void){
    if(!config_loaded) return CONFIG_FAIL;
    #define APPLY_SET(module, field) config_entrances.field = module##_GET_CONFIG();
    CONFIG_MODULE_LIST(APPLY_SET)
    #undef APPLY_SET
    config_entrances.version = CONFIG_VERSION;
    return CONFIG_OKAY;
}


static CONFIG_RETURN_TYPE config_get_defaults_global(void){
    #define APPLY_SET(module, field) config_entrances.field = module##_GET_DEFAULT_CONFIG();
    CONFIG_MODULE_LIST(APPLY_SET)
    #undef APPLY_SET
    config_entrances.version = CONFIG_VERSION;
    return CONFIG_OKAY;
}