#include "common.h"


#ifndef SERIAL_PARSER_H
#define SERIAL_PARSER_H

typedef enum{
    SERIAL_PARSER_OKAY,
    SERIAL_PARSER_FAIL
} SERIAL_PARSER_RETURN_TYPE;

SERIAL_PARSER_RETURN_TYPE SERIAL_PARSER_PARSE(uint8_t* buffer, uint32_t len);

#endif