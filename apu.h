#ifndef APU_H_INCLUDED
#define APU_H_INCLUDED

#include "M6502/M6502.h"
#include <stdint.h>

typedef void (*mapperinit_t)(void);
typedef void (*mappercleanup_t)(void);
typedef int (*mapperwrite_t)(word addr, byte data);
typedef int (*mapperread_t)(word addr);
typedef void (*mapperreset_t)(void);
typedef void (*mapperprocess_t)(int cycles);
typedef int32_t (*mapperoutput_t)(void);

typedef struct apumapper_s
{
    mapperinit_t init;
    mappercleanup_t cleanup;
    mapperreset_t reset;        // resets all mapper states and registers to default
    mapperwrite_t write;        // attempt to write to this mapper. return 1 if address is in range
    mapperread_t read;          // attempt to read from this mapper. return -1 if address not in range.
    mapperprocess_t process;    // process x cycles for this mapper. returns a signed 32 bit sample of the output
    mapperoutput_t output;
}apumapper_t;

#define APU_NTSC 0
#define APU_PAL 1

#define BIT(v, b) (((v>>b)&1) == 1)

struct apu_s;

struct apu_s* apu_create(int samplerate, byte clockstandard);
void apu_destroy(struct apu_s* apu);
void apu_setcontext(struct apu_s* apu);
void apu_write(word addr, byte data);
byte apu_read(word addr);
void apu_process(uint32_t cpu_cycles);
int32_t apu_output(void);
void apu_reset(byte snd_mappers);



#endif // APU_H_INCLUDED
