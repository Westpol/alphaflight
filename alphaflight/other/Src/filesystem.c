#include "filesystem.h"
#include "sd.h"

superblock sb = {0};
metadata mb[SD_USABLE_BLOCK_SIZE_BYTES / sizeof(metadata)] = {0};

