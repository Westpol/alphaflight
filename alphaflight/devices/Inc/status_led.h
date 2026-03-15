#include "common.h"
#include "scheduler.h"

#ifndef STATUS_LED_H
#define STATUS_LED_H

void STATUS_LED_INIT(void);
void STATUS_LED_SET_R(uint32_t brightness);
void STATUS_LED_SET_G(uint32_t brightness);
void STATUS_LED_SET_B(uint32_t brightness);
void STATUS_LED_SET_ALL(uint32_t brightness);

#endif