#include "apu.h"
#include <malloc.h>
#include <string.h>


extern M6502 cpu;

#define APU_PULSE1DUTYVOL   0x4000
#define APU_PULSE1SWEEP     0x4001
#define APU_PULSE1TMRL      0x4002
#define APU_PULSE1TMRH      0x4003
#define APU_PULSE2DUTYVOL   0x4004
#define APU_PULSE2SWEEP     0x4005
#define APU_PULSE2TMRL      0x4006
#define APU_PULSE2TMRH      0x4007
#define APU_TRICOUNTER      0x4008
#define APU_TRITMRL         0x400A
#define APU_TRITMRH         0x400B
#define APU_NOISEVOL        0x400C
#define APU_NOISEPERIOD     0x400E
#define APU_NOISELCL        0x400F
#define APU_DMCIRQ          0x4010
#define APU_DMCCOUNTER      0x4011
#define APU_DMCADDR         0x4012
#define APU_DMCLENGTH       0x4013
#define APU_STATUS          0x4015
#define APU_FRAMECNTR       0x4017

struct apuenvelope_s
{
    byte loop_halt:1;
    byte const_vol:1;
    byte volperiod:4;
    byte counter:4;
    byte divider:5;
    byte start:1;
    byte out:4;
};
struct apupulse_s
{
    byte duty:2;            // duty cycle of the pulse
    struct apuenvelope_s env;

    byte sweep_enable:1;
    byte sweep_period:3;
    byte sweep_negate:1;
    byte sweep_shift:3;
    byte sweep_divider:4;
    word sweep_target:12;
    byte sweep_silence:1;
    byte sweep_reload:1;

    word timer_period:11;      // controls the frequency of the pulse
    byte counter;            // length counter; when reaches zero, channel is silenced

    word timer:12;             // set to timer_period * 8 (left shift) when timer period set, or this overflows
    byte phase:3;              // pulse wave phase/sequence

    byte enabled:1;
};

const byte pulseseq[4][8] =
{
    { 0, 1, 0, 0, 0, 0, 0, 0 },     // 12.5%
    { 0, 1, 1, 0, 0, 0, 0, 0 },     // 25%
    { 0, 1, 1, 1, 1, 0, 0, 0 },     // 50%
    { 1, 0, 0, 1, 1, 1, 1, 1 }      // 25% negated
};

struct aputri_s
{
    byte control:1;
    byte halt:1;
    byte cnt_reload:7;          // linear counter reload value

    word timer_period:11;       // period for the wave frequency timer
    byte counter;             // length counter
    byte lincount:7;            // linear counter
    word timer:11;              // sequence timer
    byte phase:5;               // triangle wave phase/sequence

    byte enabled:1;
};

const byte triseq[32] =
{
    15, 14, 13, 12, 11, 10,  9,  8,
     7,  6,  5,  4,  3,  2,  1,  0,
     0,  1,  2,  3,  4,  5,  6,  7,
     8,  9, 10, 11, 12, 13, 14, 15
};

struct apunoise_s
{
    struct apuenvelope_s env;

    byte mode:1;
    byte period:4;
    word period_actual:12;
    word timer:12;
    byte counter;
    word shiftreg:15;           // shift reg for noise

    byte enabled:1;
};

const word noise_periods_ntsc[16] =
{
     4, 8, 16, 32, 64, 96, 128, 160, 202, 254, 380, 508, 762, 1016, 2034, 4068
};
const word noise_periods_pal[16] =
{
     4, 7, 14, 30, 60, 88, 118, 148, 188, 236, 354, 472, 708,  944, 1890, 3778
};

struct apudmc_s
{
    byte irq:1;
    byte loop:1;
    byte rate:4;
    word rate_actual:9;
    word timer:9;

    // memory reader
    word address;
    word addresscur;
    word length:12;
    word bytesleft:12;
    byte sample;
    byte buffered:1;

    // output unit
    byte counter:7;
    byte shiftreg;
    byte bitsleft;
    byte silence;

    byte control:1;
};

const word dmc_periods_ntsc[16] =
{
    428, 380, 340, 320, 286, 254, 226, 214, 190, 160, 142, 128, 106,  84,  72,  54
};
const word dmc_periods_pal[16] =
{
    398, 354, 316, 298, 276, 236, 210, 198, 176, 148, 132, 118,  98,  78,  66,  50
};

const byte length_lut[32] =
{
    10,254, 20,  2, 40,  4, 80,  6, 160,  8, 60, 10, 14, 12, 26, 14,
    12, 16, 24, 18, 48, 20, 96, 22, 192, 24, 72, 26, 16, 28, 32, 30
};

struct apu_s
{
    byte regs[(APU_FRAMECNTR - APU_PULSE1DUTYVOL)+1];
    struct apupulse_s pulse1;
    struct apupulse_s pulse2;
    struct aputri_s   tri;
    struct apunoise_s noise;
    struct apudmc_s   dmc;
    struct apuframecnt_s
    {
        byte mode:1;
        byte interrupt:1;
        byte int_inhibit:1;
        byte updated:1;
        word count:15;
    }framecnt;

    uint32_t cpu_cycles;

    struct apuenvelope_s* envelopes[3];
    uint32_t cpu_clock;

    uint32_t clock_cycles_per_sample;   // fixed point 8 bit fractional
    const word* noise_periods;
    const word* dmc_periods;
    uint32_t pulse_mix_lut[31];
    uint32_t tnd_mix_lut[203];

}*apuctx;


#define CPU_CLOCK_NTSC 1789773L // Hz
#define CPU_CLOCK_PAL  1662607L // Hz


struct apu_s* apu_create(int samplerate, byte clockstandard)
{
    struct apu_s* apu = malloc(sizeof(struct apu_s));

    if(!apu) return NULL;

    if(clockstandard == APU_NTSC)
    {
        apu->cpu_clock = CPU_CLOCK_NTSC;
        apu->noise_periods = noise_periods_ntsc;
        apu->dmc_periods = dmc_periods_ntsc;
    }
    else
    if(clockstandard == APU_PAL)
    {
        apu->cpu_clock = CPU_CLOCK_PAL;
        apu->noise_periods = noise_periods_pal;
        apu->dmc_periods = dmc_periods_pal;
    }
    else
    {
        apu->cpu_clock = CPU_CLOCK_NTSC;
        apu->noise_periods = noise_periods_ntsc;
        apu->dmc_periods = dmc_periods_ntsc;
    }

    apu->clock_cycles_per_sample = (uint32_t) ((((uint64_t)apu->cpu_clock)<<16) / samplerate);

    int n;
    for(n = 0; n < 31; ++n)
    {
        apu->pulse_mix_lut[n] = (uint32_t)((95.52 / (8128.0 / (double)n + 100.0)) * 0xFFFFFFFFUL);
    }

    for(n = 0; n < 203; ++n)
    {
        apu->tnd_mix_lut[n] = (uint32_t)((163.67 / (24329.0 / (double)n + 100.0)) * 0xFFFFFFFFUL);
    }

    apu->envelopes[0] = &apu->pulse1.env;
    apu->envelopes[1] = &apu->pulse2.env;
    apu->envelopes[2] = &apu->noise.env;

    return apu;
}

void apu_destroy(struct apu_s* apu)
{
    if(apu == NULL)
        apu = apuctx;

    if(apuctx == apu)
        apuctx = NULL;

    free(apu);
}

void apu_setcontext(struct apu_s* apu)
{
    if(apu != NULL)
        apuctx = apu;
}

void apu_quarter_frame(void);
void apu_half_frame(void);

void apu_calcsweep_pulse1(void)
{
    if(apuctx->pulse1.sweep_negate)
        apuctx->pulse1.sweep_target = apuctx->pulse1.timer_period - ((apuctx->pulse1.timer_period>>apuctx->pulse1.sweep_shift)-1);
    else
        apuctx->pulse1.sweep_target = apuctx->pulse1.timer_period + (apuctx->pulse1.timer_period>>apuctx->pulse1.sweep_shift);

    if((apuctx->pulse1.timer_period < 8) || (apuctx->pulse1.sweep_target > 0x7FF))
        apuctx->pulse1.sweep_silence = 1;
    else
        apuctx->pulse1.sweep_silence = 0;
}

void apu_calcsweep_pulse2(void)
{
    if(apuctx->pulse2.sweep_negate)
        apuctx->pulse2.sweep_target = apuctx->pulse2.timer_period - (apuctx->pulse2.timer_period>>apuctx->pulse2.sweep_shift);
    else
        apuctx->pulse2.sweep_target = apuctx->pulse2.timer_period + (apuctx->pulse2.timer_period>>apuctx->pulse2.sweep_shift);

    if((apuctx->pulse2.timer_period < 8) || (apuctx->pulse2.sweep_target > 0x7FF))
        apuctx->pulse2.sweep_silence = 1;
    else
        apuctx->pulse2.sweep_silence = 0;
}

void apu_write(word addr, byte data)
{
    switch(addr)
    {
        // basic APU registers
        case APU_PULSE1DUTYVOL:
            apuctx->regs[addr&0x1F] = data;
            apuctx->pulse1.duty             = data>>6;
            apuctx->pulse1.env.loop_halt    = BIT(data, 5);//data&0x20;
            apuctx->pulse1.env.const_vol    = BIT(data, 4);//data&0x10;
            apuctx->pulse1.env.volperiod    = data&0xF;
            break;
        case APU_PULSE1SWEEP:
            apuctx->regs[addr&0x1F] = data;
            apuctx->pulse1.sweep_enable = BIT(data, 7);//data&0x80;
            apuctx->pulse1.sweep_period = (data>>4)&0x07;
            apuctx->pulse1.sweep_negate = BIT(data, 3);//data&0x08;
            apuctx->pulse1.sweep_shift = data&0x07;
            apuctx->pulse1.sweep_reload = 1;
            break;
        case APU_PULSE1TMRL:
            apuctx->regs[addr&0x1F] = data;
            apuctx->pulse1.timer_period = (apuctx->pulse1.timer_period&0x0700)|data;
            apu_calcsweep_pulse1();
            break;
        case APU_PULSE1TMRH:
            apuctx->regs[addr&0x1F] = data;
            apuctx->pulse1.timer_period = (apuctx->pulse1.timer_period&0x00FF)|((data&0x07)<<8);
            apu_calcsweep_pulse1();
            if(apuctx->pulse1.enabled) apuctx->pulse1.counter = length_lut[data>>3];
            apuctx->pulse1.phase = 0;       // reset phase/sequence
            apuctx->pulse1.env.start = 1;
            break;
        case APU_PULSE2DUTYVOL:
            apuctx->regs[addr&0x1F] = data;
            apuctx->pulse2.duty             = data>>6;
            apuctx->pulse2.env.loop_halt    = BIT(data, 5);//data&0x20;
            apuctx->pulse2.env.const_vol    = BIT(data, 4);//data&0x10;
            apuctx->pulse2.env.volperiod    = data&0xF;
            break;
        case APU_PULSE2SWEEP:
            apuctx->regs[addr&0x1F] = data;
            apuctx->pulse2.sweep_enable = BIT(data, 7);//data&0x80;
            apuctx->pulse2.sweep_period = (data>>4)&0x07;
            apuctx->pulse2.sweep_negate = BIT(data, 3);//data&0x08;
            apuctx->pulse2.sweep_shift = data&0x07;
            apuctx->pulse2.sweep_reload = 1;
            break;
        case APU_PULSE2TMRL:
            apuctx->regs[addr&0x1F] = data;
            apuctx->pulse2.timer_period = (apuctx->pulse2.timer_period&0x0700)|data;
            apu_calcsweep_pulse2();
            break;
        case APU_PULSE2TMRH:
            apuctx->regs[addr&0x1F] = data;
            apuctx->pulse2.timer_period = (apuctx->pulse2.timer_period&0x00FF)|((data&0x07)<<8);
            apu_calcsweep_pulse2();
            if(apuctx->pulse2.enabled) apuctx->pulse2.counter = length_lut[data>>3];
            apuctx->pulse2.phase = 0;       // reset phase/sequence
            apuctx->pulse2.env.start = 1;
            break;
        case APU_TRICOUNTER:
            apuctx->regs[addr&0x1F] = data;
            apuctx->tri.control = BIT(data, 7);//data&0x80;
            if(apuctx->tri.control)
                apuctx->tri.halt = 1;
            apuctx->tri.cnt_reload = data&0x7f;
            break;
        case APU_TRITMRL:
            apuctx->regs[addr&0x1F] = data;
            apuctx->tri.timer_period = (apuctx->tri.timer_period&0x0700)|data;
            break;
        case APU_TRITMRH:
            apuctx->regs[addr&0x1F] = data;
            apuctx->tri.timer_period = (apuctx->tri.timer_period&0x00FF)|((data&0x07)<<8);
            if(apuctx->tri.enabled) apuctx->tri.counter = length_lut[data>>3];
            apuctx->tri.halt = 1;    // set halt flag
            break;
        case APU_NOISEVOL:
            apuctx->regs[addr&0x1F] = data;
            apuctx->noise.env.loop_halt = BIT(data, 5);//data&0x20;
            apuctx->noise.env.const_vol = BIT(data, 4);//data&0x10;
            apuctx->noise.env.volperiod = data&0x0F;
            break;
        case APU_NOISEPERIOD:
            apuctx->regs[addr&0x1F] = data;
            apuctx->noise.mode = BIT(data, 7);//data&0x80;
            apuctx->noise.period = data&0x0F;
            apuctx->noise.period_actual = apuctx->noise_periods[apuctx->noise.period];
            break;
        case APU_NOISELCL:
            apuctx->regs[addr&0x1F] = data;
            if(apuctx->noise.enabled) apuctx->noise.counter = length_lut[data>>3];
            apuctx->noise.env.start = 1;
            break;
        case APU_DMCIRQ:
            apuctx->regs[addr&0x1F] = data;
            apuctx->dmc.irq = BIT(data, 7);//data&0x80;
            apuctx->dmc.loop = BIT(data, 6);//data&0x40;
            apuctx->dmc.rate = data&0x0f;
            apuctx->dmc.rate_actual = apuctx->dmc_periods[apuctx->dmc.rate];
            break;
        case APU_DMCCOUNTER:
            apuctx->regs[addr&0x1F] = data;
            apuctx->dmc.counter = data&0x7f;
            break;
        case APU_DMCADDR:
            apuctx->regs[addr&0x1F] = data;
            apuctx->dmc.address = (data<<6) | 0xC000;
            apuctx->dmc.addresscur = apuctx->dmc.address;
            break;
        case APU_DMCLENGTH:
            apuctx->regs[addr&0x1F] = data;
            apuctx->dmc.length = (data<<4) | 1;
            apuctx->dmc.bytesleft = apuctx->dmc.length;
            break;
        case APU_STATUS:
            apuctx->regs[addr&0x1F] = data;
            apuctx->dmc.control = BIT(data, 4);//(data>>4)&1;
            apuctx->noise.enabled = BIT(data, 3);//data&0x08;
            apuctx->tri.enabled =   BIT(data, 2);//data&0x04;
            apuctx->pulse2.enabled = BIT(data, 1);//(data>>1)&1;
            apuctx->pulse1.enabled = BIT(data, 0);//data&1;
            break;
        case APU_FRAMECNTR:
            apuctx->regs[addr&0x1F] = data;
            apuctx->framecnt.count = 0;
            apuctx->framecnt.mode = BIT(data, 7);//data&0x80;
            apuctx->framecnt.int_inhibit = BIT(data, 6);//data&0x40;
            apuctx->framecnt.updated = 1;
            break;
    }
}

byte apu_read(word addr)
{
    if(addr == APU_STATUS)
    {
        return (apuctx->dmc.irq<<7) | (apuctx->framecnt.interrupt<<6) | ((apuctx->noise.counter>0)<<3) | ((apuctx->tri.counter>0)<<2) | ((apuctx->pulse2.counter>0)<<1) | (apuctx->pulse1.counter>0);
    }
    return 0;
}

void apu_quarter_frame(void)
{
    // process envelopes (pulses and noise)
    struct apuenvelope_s* env;
    int i;

    for(i = 0; i < 3; ++i)
    {
        env = apuctx->envelopes[i];

        if(env->start)
        {
            env->start = 0;

            env->counter = 15;
            env->divider = env->volperiod+1;
        }
        else
        {

            if(!(--env->divider))
            {
                env->divider = env->volperiod+1;

                if(env->counter)
                    --env->counter;
                else
                if(env->loop_halt)
                {
                    env->counter = 15;
                }
            }
        }

        if(env->const_vol)
            env->out = env->volperiod;
        else
            env->out = env->counter;
    }

    // process triangle's linear counter
    if(apuctx->tri.halt)
        apuctx->tri.lincount = apuctx->tri.cnt_reload;
    else
    if(apuctx->tri.lincount)
        --apuctx->tri.lincount;

    if(!apuctx->tri.control)
        apuctx->tri.halt = 0;
}

void apu_half_frame(void)
{
    // process length counters of pulses, triangle, noise, and DMC
    if(apuctx->pulse1.enabled)
    {
        if(apuctx->pulse1.counter && (!apuctx->pulse1.env.loop_halt))
        {
            --apuctx->pulse1.counter;
        }
    }
    else
    {
        apuctx->pulse1.counter = 0;
    }

    if(apuctx->pulse2.enabled)
    {
        if(apuctx->pulse2.counter && (!apuctx->pulse2.env.loop_halt))
        {
            --apuctx->pulse2.counter;
        }
    }
    else
    {
        apuctx->pulse2.counter = 0;
    }

    if(apuctx->tri.enabled)
    {
        if(apuctx->tri.counter && (!apuctx->tri.halt))
        {
            --apuctx->tri.counter;
        }
    }
    else
    {
        apuctx->tri.counter = 0;
    }

    if(apuctx->noise.enabled)
    {
        if(apuctx->noise.counter && (!apuctx->noise.env.loop_halt))
        {
            --apuctx->noise.counter;
        }
    }
    else
    {
        apuctx->noise.counter = 0;
    }

    if(apuctx->pulse1.sweep_divider)
    {
        // sweep units
        --apuctx->pulse1.sweep_divider;
        if(apuctx->pulse1.sweep_reload)
        {
            apuctx->pulse1.sweep_reload = 0;
            apuctx->pulse1.sweep_divider = apuctx->pulse1.sweep_period+1;
        }
    }
    else
    {
        if(apuctx->pulse1.sweep_enable && apuctx->pulse1.sweep_shift)
        {
     //   apuctx->pulse1.sweep_reload = 0;
            apuctx->pulse1.sweep_divider = apuctx->pulse1.sweep_period+1;

            apuctx->pulse1.timer_period = apuctx->pulse1.sweep_target;
            apu_calcsweep_pulse1();
        }
    }

    if(apuctx->pulse2.sweep_divider)
    {
        // sweep units
        --apuctx->pulse2.sweep_divider;
        if(apuctx->pulse2.sweep_reload)
        {
            apuctx->pulse2.sweep_reload = 0;
            apuctx->pulse2.sweep_divider = apuctx->pulse2.sweep_period+1;
        }
    }
    else
    {
        if(apuctx->pulse2.sweep_enable && apuctx->pulse2.sweep_shift)
        {
          //  apuctx->pulse2.sweep_reload = 0;
            apuctx->pulse2.sweep_divider = apuctx->pulse2.sweep_period+1;

            apuctx->pulse2.timer_period = apuctx->pulse2.sweep_target;
            apu_calcsweep_pulse2();
        }
    }
}

inline void apu_noisegen(void)
{
    word feedback = (apuctx->noise.mode) ? ((apuctx->noise.shiftreg&1)^((apuctx->noise.shiftreg>>5)&1)) : ((apuctx->noise.shiftreg&1)^((apuctx->noise.shiftreg>>1)&1))<<14;

    apuctx->noise.shiftreg = (apuctx->noise.shiftreg>>1) | feedback;
}

void apu_process(uint32_t cpu_cycles)
{
    uint32_t c;

    for(c = 0; c < cpu_cycles; ++c)
    {
        if((c&1) || apuctx->framecnt.updated)
        {
            // clock frame counter
            if(apuctx->framecnt.mode)
            {
                // 5 step sequence
                if(apuctx->framecnt.updated||(apuctx->framecnt.count == 7456)||(apuctx->framecnt.count == 18640))
                {
                    // "quarter frame" and "half frame"
                    apu_quarter_frame();
                    apu_half_frame();
                    if(apuctx->framecnt.updated)
                    {
                        apuctx->framecnt.updated = 0;
                    }
                }
                else
                if((apuctx->framecnt.count == 3728)||(apuctx->framecnt.count == 11185))
                {
                    // "quarter frame"
                    apu_quarter_frame();
                }

                if(apuctx->framecnt.count == 18640)
                    apuctx->framecnt.count = 0;
                else
                    ++apuctx->framecnt.count;
            }
            else
            {
                // 4 step sequence
                if(apuctx->framecnt.updated||(apuctx->framecnt.count == 7456)||(apuctx->framecnt.count == 14914))
                {
                    // "quarter frame" and "half frame"
                    apu_quarter_frame();
                    apu_half_frame();
                    if(apuctx->framecnt.updated)
                    {
                        apuctx->framecnt.updated = 0;
                    }
                }
                else
                if((apuctx->framecnt.count == 3728)||(apuctx->framecnt.count == 11185))
                {
                    // "quarter frame"
                    apu_quarter_frame();
                }

                if(apuctx->framecnt.count == 14914)
                {
                    apuctx->framecnt.count = 0;
                    if(!apuctx->framecnt.int_inhibit)
                    {
                        apuctx->framecnt.interrupt = 1;
                      //  Int6502(&cpu, INT_IRQ);
                      //  Run6502(&cpu);
                    }
                }
                else
                    ++apuctx->framecnt.count;
            }
        }

        if(c&1)
        {
            // process timers for all waves
            if(apuctx->pulse1.timer)
                --apuctx->pulse1.timer;
            else
            {
                apuctx->pulse1.timer = apuctx->pulse1.timer_period;
                if(apuctx->pulse1.phase)
                    --apuctx->pulse1.phase;
                else
                    apuctx->pulse1.phase = 7;
            }

            if(apuctx->pulse2.timer)
                --apuctx->pulse2.timer;
            else
            {
                apuctx->pulse2.timer = apuctx->pulse2.timer_period;
                if(apuctx->pulse2.phase)
                    --apuctx->pulse2.phase;
                else
                    apuctx->pulse2.phase = 7;
            }

            if(apuctx->noise.timer)
                --apuctx->noise.timer;
            else
            {
                apuctx->noise.timer = apuctx->noise.period_actual;
                // calculate noise
                apu_noisegen();
            }
        }

        // DMC shite
        if(apuctx->dmc.control)
        {
            if(!apuctx->dmc.buffered && apuctx->dmc.bytesleft)
            {
                apuctx->dmc.sample = Rd6502(apuctx->dmc.addresscur);

                if(!(--apuctx->dmc.bytesleft))
                {
                    if(apuctx->dmc.loop)
                    {
                        apuctx->dmc.addresscur = apuctx->dmc.address;
                        apuctx->dmc.bytesleft = apuctx->dmc.length;
                    }
                    else
                        apuctx->dmc.irq = 1;
                }
                else
                if(apuctx->dmc.addresscur == 0xFFFF)
                    apuctx->dmc.addresscur = 0x8000;
                else
                ++apuctx->dmc.addresscur;

                apuctx->dmc.buffered = 1;
            }
        }


        if(apuctx->dmc.timer)
            --apuctx->dmc.timer;
        else
        {
            apuctx->dmc.timer = apuctx->dmc.rate_actual;

            if(!apuctx->dmc.bitsleft)
            {
                apuctx->dmc.bitsleft = 8;
                if(!apuctx->dmc.buffered)
                    apuctx->dmc.silence = 1;
                else
                {
                    apuctx->dmc.silence = 0;
                    apuctx->dmc.shiftreg = apuctx->dmc.sample;
                    apuctx->dmc.buffered = 0;
                }
            }

            if(!apuctx->dmc.silence)
            {
                if((apuctx->dmc.counter>1) && !(apuctx->dmc.shiftreg&1))
                    apuctx->dmc.counter-=2;
                else
                if((apuctx->dmc.counter<126) && (apuctx->dmc.shiftreg&1))
                    apuctx->dmc.counter+=2;
            }

            apuctx->dmc.shiftreg>>=1;
            --apuctx->dmc.bitsleft;
        }

        // clocked on every cycle
        if((apuctx->tri.lincount>0) && (apuctx->tri.counter>0))
        {
            if(apuctx->tri.timer)
                --apuctx->tri.timer;
            else
            {
                apuctx->tri.timer = apuctx->tri.timer_period;
                if(apuctx->tri.phase)
                    --apuctx->tri.phase;
                else
                    apuctx->tri.phase = 31;
            }
        }
    }
}

int32_t apu_output(void)
{
    if(!apuctx) return 0;

    apuctx->cpu_cycles += apuctx->clock_cycles_per_sample;

    apu_process(apuctx->cpu_cycles>>16);
    apuctx->cpu_cycles &= 0xFFFF;

    byte pulse1, pulse2;
    byte tri;
    byte noise;
    byte dmc;

    if(!apuctx->pulse1.counter || apuctx->pulse1.sweep_silence)
        pulse1 = 0;
    else
    {
        pulse1 = pulseseq[apuctx->pulse1.duty][apuctx->pulse1.phase] * apuctx->pulse1.env.out * (apuctx->pulse1.counter>0);
    }

    if(!apuctx->pulse2.counter || apuctx->pulse2.sweep_silence)
        pulse2 = 0;
    else
    {
        pulse2 = pulseseq[apuctx->pulse2.duty][apuctx->pulse2.phase] * apuctx->pulse2.env.out * (apuctx->pulse2.counter>0);
    }

    if(apuctx->tri.timer_period < 2)
        tri = 7;
    else
        tri = triseq[apuctx->tri.phase];

    noise = (apuctx->noise.shiftreg&1) * apuctx->noise.env.out * (apuctx->noise.counter>0);
    dmc = apuctx->dmc.counter;

                                            // "tri + tri<<1" (tri + tri*2 == tri*3) + noise<<1 (noise*2) + dmc
    return (int32_t)((int64_t)(apuctx->pulse_mix_lut[pulse1 + pulse2] + apuctx->tnd_mix_lut[(tri + (tri<<1)) + (noise<<1) + dmc]) - 0x7fffffffL);
}

void apu_reset(byte snd_mappers)
{
    if(!apuctx) return;

    memset(apuctx, 0, (((int)&apuctx->envelopes) - (int)apuctx));

    int i = 0;
    for(i = 0x00; i <= 0x13; ++i)
    {
        apu_write(i, 0x00);
    }

    apu_write(APU_STATUS, 0x0f);
    apu_write(APU_FRAMECNTR, 0x40);

    apuctx->noise.shiftreg = 1;
}

