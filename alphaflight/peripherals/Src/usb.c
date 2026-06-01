#include "usb.h"
#include "stm32h723xx.h"
#include "usbd_cdc_if.h"
#include "timer.h"

#include "lsm6dso.h"
#include "usbd_def.h"

#include "serial_parser.h"

#include "math_types.h"
#include <string.h>

volatile static bool connection_detected = false;

volatile static uint32_t usb_rx_len = 0;
volatile static bool usb_new_rx = false;
static uint8_t usb_rx_buffer[APP_RX_DATA_SIZE] = {0};


void USB_PRINTLN(const char *format, ...){
    if(!connection_detected) return;
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
    if(!connection_detected) return 0;
    //const task_stat_t* stat = SCHEDULER_GET_TASK_STAT_BY_INDEX(0);
    //USB_PRINTLN("%luus TEST | task: %s, task time: %d", MICROS32(), task->task_name,stat->average_exec_time);
    //USB_PRINTLN("Data Length: %d", LL_DMA_GetDataLength(DMA2, LL_DMA_STREAM_0));
    IMU_T imu = IMU_GET_RAW_DATA();


    float angle = UTILS_RADIANS(10) / 2.0;
    QUAT_T q_set = {cosf(angle), 1*sinf(angle), 0.0f, 0.0f};
    QUAT_T gravity = {0.0f, 0.0f, 0.0f, 1.0f};

    QUAT_T q_setpoint = UTILS_QUATERNION_NORMALIZE(UTILS_QUATERNION_PRODUCT(UTILS_QUATERNION_PRODUCT(q_set, gravity), UTILS_QUATERNION_CONJUGATE(q_set)));
    VECT_3D_T v_setpoint = {q_setpoint.x, q_setpoint.y, q_setpoint.z};
    
    QUAT_T q_curr = imu.processed.quat;
    QUAT_T q_current = UTILS_QUATERNION_NORMALIZE(UTILS_QUATERNION_PRODUCT(UTILS_QUATERNION_PRODUCT(q_curr, gravity), UTILS_QUATERNION_CONJUGATE(q_curr)));
    VECT_3D_T v_current = {q_current.x, q_current.y, q_current.z};

    VECT_3D_T v_error = UTILS_VECT_CROSS_PRODUCT(v_current, v_setpoint);

    VECT_3D_T e = imu.debug.e;
    VECT_3D_T e_i = imu.debug.e_i;

    //USB_PRINTLN("e:(%10f,%10f,%10f) e_i:(%10f, %10f, %10f) | %f,%f", e.x, e.y, e.z, e_i.x, e_i.y, e_i.z, UTILS_DEGREES(v_error.x), UTILS_DEGREES(v_error.y));
    USB_PRINTLN("Pitch: %f°\nRoll: %f°", UTILS_DEGREES(imu.processed.attitude.pitch), UTILS_DEGREES(imu.processed.attitude.roll));
    //USB_PRINTLN("(%f, %f, %f)", imu.debug.g_est.x, imu.debug.g_est.y, imu.debug.g_est.z);

    //USB_PRINTLN("%f,%f,%f,%f | %f,%f", imu.processed.quat.w, imu.processed.quat.x, imu.processed.quat.y, imu.processed.quat.z, UTILS_DEGREES(v_error.x), UTILS_DEGREES(v_error.y));
    return 0;
}

void USB_PRINTLN_BLOCKING(const char *format, ...){
    if(!connection_detected) return;
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

void USB_CHANGE_CONSOLE_STATUS(bool connection){
    connection_detected = connection;
}

void UTIL_USB_PRINT(const char *format, ...){

}

void UTIL_USB_PRINT_HEX(uint8_t *data, uint32_t len){

}

void UTIL_USB_PRINT_RAW(const char* message, uint32_t len){

}

uint32_t USB_RECIEVE_PARSE_DATA(const task_info_t* task){
    SERIAL_PARSER_PARSE(usb_rx_buffer, usb_rx_len);
    usb_new_rx = false;

    return 0;
}

void USB_RECIEVE_INTERRUPT(uint8_t* buf, uint32_t len){

    if(usb_new_rx) return;

    if(len > APP_RX_DATA_SIZE){
        len = APP_RX_DATA_SIZE;
    }

    memcpy(usb_rx_buffer, buf, len);

    usb_rx_len = len;
    usb_new_rx = true;

    SCHEDULER_REGISTER_THROWAWAY_TASK(USB_RECIEVE_PARSE_DATA, 100, "USB RX Parsing");
}