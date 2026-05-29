#include "serial_parser.h"

#include <string.h>

#include "config.h"
#include "usb.h"

SERIAL_PARSER_RETURN_TYPE SERIAL_PARSER_PARSE(uint8_t* buffer){

    if(strncmp((const char*)buffer, "tasks\n", sizeof(buffer)) == 0){
        SCHEDULER_PRINT_TASK_PAGE_SINGLE();
    }else if(strncmp((const char*)buffer, "debug\n", sizeof(buffer)) == 0){
        USB_STATUS(NULL);
    }else if(strncmp((const char*)buffer, "save\n", sizeof(buffer)) == 0){
        if(CONFIG_STORE_TO_SD() == CONFIG_OKAY){
            USB_PRINTLN("Successfully saved config to SD card!");
        }else{
            USB_PRINTLN("Save failed! Retry or all changes will be lost upon reboot!");
        }
    }else if(strncmp((const char*)buffer, "load defaults\n", sizeof(buffer)) == 0){
        if(CONFIG_LOAD_DEFAULTS() == CONFIG_OKAY){
            USB_PRINTLN("Successfully loaded defaults. Changes not persistent, use 'save' to keep changes!");
        }else{
            USB_PRINTLN("Loading defaults failed!");
        }
    }
    else{
        USB_PRINTLN("Command not found!");
    }

    return SERIAL_PARSER_OKAY;
}