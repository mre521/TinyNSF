#ifndef AUDIOCONFIG_H_INCLUDED
#define AUDIOCONFIG_H_INCLUDED
#include <stdint.h>

typedef struct
{
    uint32_t       frequency;
    int            bits;
    int            channels;
    int            encoding;
    uint32_t       bufSize;
} AudioConfig;

#endif  // AUDIOCONFIG_H_INCLUDED
