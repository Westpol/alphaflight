#include "common.h"

#ifndef MIXER_H
#define MIXER_H

typedef enum{
    MIXER_FAIL,
    MIXER_OKAY
} MIXER_RETURN_TYPE;

MIXER_RETURN_TYPE MIXER_SET_POSITION(uint16_t elevator, uint16_t aileron);

#endif