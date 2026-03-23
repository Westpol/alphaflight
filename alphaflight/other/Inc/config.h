#include "common.h"
#include "lsm6dso.h"

#ifndef CONFIG_H
#define CONFIG_H

#define CONFIG_SPACE_BLOCKS 80  // 320 kib of config space
#define CONFIG_SPACE_START_BLOCK 19 // blocks 19-99 = config block, block 99 = super block, 100... = (meta)data

#define CONFIG_VERSION 0

typedef struct{
    imu_config_t imu;
}config_entrances;

#endif