#include "serial_parser.h"

#include <stdint.h>
#include <string.h>

#include "usbd_cdc_if.h"
#include "config.h"
#include "usb.h"
#include "gps.h"
#include "bmp390.h"

static const char* commands[] = {"tasks\n", "debug\n", "save\n", "load default\n", "show gps\n", "show baro\n", "help\n"};
static const char* getset[] = {"get", "set"};
static const char* devices[] = {"imu", "baro", "gps", "crsf", "servo", "osd", "logger"}
static const char* points[][] = {{"rate"}, {"rate", "offset"}, {"rate", "protocol"}, {"channel throttle", "channel pitch", "channel roll", "telemetry"}, {"aileron left", "aileron right", "pitch", "min", "max", "center", "direction"}, {"enable"}, {"rate", "log data"}};

static SERIAL_PARSER_RETURN_TYPE print_help();

SERIAL_PARSER_RETURN_TYPE SERIAL_PARSER_PARSE(uint8_t* buffer, uint32_t len){

    //-------------------- check for fixed commands -------------------------
    int16_t command_type = -1;
    for(uint16_t i = 0; i < sizeof(commands) / sizeof(commands[0]); i++){
        if(strncmp((const char*)buffer, commands[i], UTILS_MIN_I(strlen(commands[i]), len)) == 0){    // strlen(commands[i]) - 1 to cut off \n in order to successfully parse get/set command
            command_type = i;
            break;
        }
    }

    switch (command_type) {
        case 0:
            SCHEDULER_PRINT_TASK_PAGE_SINGLE();
        break;

        case 1:
            USB_STATUS(NULL);
        break;

        case 2:
            if(CONFIG_STORE_TO_SD() == CONFIG_OKAY){
                USB_PRINTLN("Successfully saved config to SD card!");
            } else{
                USB_PRINTLN("Save failed! Retry or else all changes will be lost upon reboot!");
            }
        break;

        case 3:
            if(CONFIG_LOAD_DEFAULTS() == CONFIG_OKAY){
                USB_PRINTLN("Successfully loaded defaults. Changes not persistent, use 'save' to keep changes!");
            } else{
                USB_PRINTLN("Loading defaults failed!");
            }
        break;

        case 4:
            GPS_PRINT_DATA();
        break;

        case 5:
            BARO_PRINT_DATA();
        break;

        case 6:
            print_help();
        break;

        case -1:
            USB_PRINTLN("Command not found!");
        break;
    }

    //------------------------- get/set ----------------------------------



    USB_PRINTLN("Command not found!");
    return SERIAL_PARSER_OKAY;
}

static SERIAL_PARSER_RETURN_TYPE print_help(){
    USB_PRINTLN("help page\n\n"
        "List of hard commands:\n"
        "help\t\t\tprints help page\n"
        "tasks\t\t\tprints all tasks and their respective CPU load\n"
        "debug\t\t\tprints debug value(s) set by user\n"
        "show [device]\t\tprints device data, devices available: [gps, baro]\n"
        "load default\t\tloads default config values in non permanent storage\n"
        "save\t\t\tsaves config data to the SD card");
}