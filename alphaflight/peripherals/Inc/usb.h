
#include "common.h"
#include <stdarg.h>
#include <stdint.h>
#include <string.h>
#include "scheduler.h"

#ifndef USB_H
#define USB_H

#define USB_PRINT_BUFFER_SIZE STM32_WORD_SIZE * 64

void USB_PRINTLN(const char *format, ...);
uint32_t USB_STATUS(const task_info_t *task);
void USB_PRINTLN_BLOCKING(const char *format, ...);
void UTIL_USB_PRINT(const char *format, ...);
void UTIL_USB_PRINT_HEX(uint8_t *data, uint32_t len);
void UTIL_USB_PRINT_RAW(const char* message, uint32_t len);

#endif