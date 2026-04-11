#include "common.h"
#include "lsm6dso.h"
#include "servo.h"
#include "dshot.h"

#ifndef CONFIG_H
#define CONFIG_H

#define CONFIG_SPACE_BLOCKS 40  // 160 kib of config space
#define CONFIG_SPACE_START_BLOCK 19 // blocks 19-59 = config block, block 98+99 = super block, 100... = (meta)data

#define CONFIG_VERSION 1

typedef enum{
    CONFIG_OKAY,
    CONFIG_FAIL
}CONFIG_RETURN_TYPE;

typedef struct{
    uint8_t version;
    imu_config_t imu;
    servo_config_t servo;
    dshot_config_t dshot;
}config_entrances_t;

CONFIG_RETURN_TYPE CONFIG_STORE_TO_SD();
CONFIG_RETURN_TYPE CONFIG_LOAD_FROM_SD();

#endif