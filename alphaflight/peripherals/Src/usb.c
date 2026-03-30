#include "usb.h"
#include "stm32h723xx.h"
#include "usbd_cdc_if.h"
#include "timer.h"

#include "lsm6dso.h"



void USB_PRINTLN(const char *format, ...){
    char message[USB_PRINT_BUFFER_SIZE];
    va_list args;
    va_start(args, format);
    int len = vsnprintf(message, sizeof(message) - 2, format, args);  // Reserve space for \r\n
    va_end(args);

    // Ensure there's space to append "\r\n"
    if (len > 0 && len < (USB_PRINT_BUFFER_SIZE - 2)) {
        message[len] = '\r';
        message[len + 1] = '\n';
        message[len + 2] = '\0'; // Null-terminate the string
        len += 2;
    }

    CDC_Transmit_HS((uint8_t *)message, len);
}

uint32_t USB_STATUS(const task_info_t *task){
    //const task_stat_t* stat = SCHEDULER_GET_TASK_STAT_BY_INDEX(0);
    //USB_PRINTLN("%luus TEST | task: %s, task time: %d", MICROS32(), task->task_name,stat->average_exec_time);
    //USB_PRINTLN("Data Length: %d", LL_DMA_GetDataLength(DMA2, LL_DMA_STREAM_0));
    IMU_PROCESSED_T imu = IMU_GET_DATA();
    USB_PRINTLN("%f°/s x",UTILS_DEGREES(imu.rate.wx));
    return 0;
}

void USB_PRINTLN_BLOCKING(const char *format, ...){
	char message[USB_PRINT_BUFFER_SIZE];
	    va_list args;
	    va_start(args, format);
	    int len = vsnprintf(message, sizeof(message) - 2, format, args);  // Reserve space for \r\n
	    va_end(args);

	    // Ensure there's space to append "\r\n"
	    if (len > 0 && len < (USB_PRINT_BUFFER_SIZE - 2)) {
	        message[len] = '\r';
	        message[len + 1] = '\n';
	        message[len + 2] = '\0'; // Null-terminate the string
	        len += 2;
	    }

	    uint32_t start = HAL_GetTick();
	    while (CDC_Transmit_HS((uint8_t *)message, len) == USBD_BUSY) {
	        if (HAL_GetTick() - start > 10) break; // Timeout after 100ms
	    }
}

void UTIL_USB_PRINT(const char *format, ...){

}

void UTIL_USB_PRINT_HEX(uint8_t *data, uint32_t len){

}

void UTIL_USB_PRINT_RAW(const char* message, uint32_t len){

}