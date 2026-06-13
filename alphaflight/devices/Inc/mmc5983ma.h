#include "common.h"
#include "spi.h"

#ifndef MMC5983MA_H
#define MMC5983MA_H

typedef enum{
    MAGNETO_OKAY,
    MAGNETO_FAIL
} MAGNETO_RETURN_TYPE;

MAGNETO_RETURN_TYPE MAGNETO_INIT(SPI_DEVICE device);

#endif