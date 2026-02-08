#include "utils.h"

uint32_t UTILS_MAX_I(uint32_t var1, uint32_t var2){
    return var1 > var2 ? var1 : var2;
}

uint32_t UTILS_MIN_I(uint32_t var1, uint32_t var2){
    return var1 < var2 ? var1 : var2;
}

float UTILS_MAX_F(float var1, float var2){
    return var1 > var2 ? var1 : var2;
}

float UTILS_MIN_F(float var1, float var2){
    return var1 < var2 ? var1 : var2;
}
