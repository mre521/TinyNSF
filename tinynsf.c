#include <stdio.h>
#include <aalib.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <pthread.h>
#include "M6502/M6502.h"
#include "apu.h"

#include <sys/ioctl.h>
#include <alsa/asoundlib.h>

#include "audioconfig.h"


#define SAMPLE_RATE 48000
#define SAMPLE_BITS 16

#define NO_AALIB

// Close an opened audio device, free any allocated buffers and
// reset any variables that reflect the current state.
void Audio_ALSA_close (snd_pcm_t *handle, void* buffer)
{
    if (handle != NULL )
    {
        snd_pcm_close(handle);
        free(buffer);
    }
}

void *Audio_ALSA_open (AudioConfig *cfg, snd_pcm_t **handle)
{
    AudioConfig tmpCfg;
    int rtn;

    if ((rtn = snd_pcm_open (handle, "default", SND_PCM_STREAM_PLAYBACK, 0)))
    {
        goto open_error;
    }

   memcpy(&tmpCfg, cfg, sizeof(AudioConfig));

    snd_pcm_hw_params_t *hw;

    if((rtn = snd_pcm_hw_params_malloc(&hw)) < 0)
    {
        goto open_error;
    }

    if((rtn = snd_pcm_hw_params_any(*handle, hw)) < 0)
    {
        goto open_error;
    }

    if((rtn = snd_pcm_hw_params_set_access(*handle, hw, SND_PCM_ACCESS_RW_INTERLEAVED)) < 0)
    {
        goto open_error;
    }

    if ( tmpCfg.bits == 8 )
    {
        if((rtn = snd_pcm_hw_params_set_format(*handle, hw, SND_PCM_FORMAT_S8)) < 0)
        {
            goto open_error;
        }
    }
    if ( tmpCfg.bits == 16 )
    {
        if((rtn = snd_pcm_hw_params_set_format(*handle, hw, SND_PCM_FORMAT_S16_LE)) < 0)
        {
            goto open_error;
        }
    }

    if((rtn = snd_pcm_hw_params_set_rate_near(*handle, hw, &tmpCfg.frequency, 0)) < 0)
    {
        goto open_error;
    }

    if((rtn = snd_pcm_hw_params_set_channels(*handle, hw, tmpCfg.channels)) < 0)
    {
        goto open_error;
    }

    if((rtn = snd_pcm_hw_params_set_buffer_size(*handle, hw, tmpCfg.bufSize*tmpCfg.bits/8)) < 0)
    {
        goto open_error;
    }

    if((rtn = snd_pcm_hw_params(*handle, hw)) < 0)
    {
        goto open_error;
    }

    snd_pcm_hw_params_free (hw);

    void* buffer = malloc(tmpCfg.bufSize*tmpCfg.bits/8);

    if (!buffer)
    {
        goto open_error;
    }

    return buffer;

open_error:
    if (*handle != NULL)
    {
        Audio_ALSA_close (*handle, buffer);
    }

    perror ("ALSA");
return NULL;
}

void *Audio_ALSA_write (snd_pcm_t *handle, void* buffer, snd_pcm_sframes_t length)
{
    if (handle == NULL)
    {
        return NULL;
    }

    snd_pcm_sframes_t remaining = length;
    snd_pcm_sframes_t written = 0;
    int8_t* buff_cur = (int8_t*)buffer;

    while(remaining > 0)
    {
        written = snd_pcm_writei (handle, (void*)buff_cur, remaining);
        if(written < 0)
        {
            snd_pcm_recover(handle, written, 1);
        }
        else if(written <= remaining)
        {
            remaining -= written;
            buff_cur += written<<(SAMPLE_BITS>>3);
        }
    }
    return (void *) buffer;
}

byte nes_wram[0x800];
byte nes_sram[0x2000];

struct nsfhead_s
{
    char id[5];     // needs to be set as 'N','E','S','M',0x1A in the file
    byte version;   // currently 1
    byte songs;     // # of songs in nsf
    byte start;     // starting song, 1 based
    word load;      // load address of data
    word init;      // init address of song
    word play;      // play address of song
    char name[32];      // name of song
    char artist[32];    // name of artist
    char copyright[32]; // copyright info
    word speedntsc;         // play speed 1/1000000th second ticks for ntsc
    byte bankswitch[8];     // bankswitch register data
    word speedpal;          // play speed 1/1000000th second ticks for pal
    byte palntsc;           // use pal, ntsc, or both
    byte extsnd;            // external sound chip support
    byte reserved[4];       // reserved for future use (right, like it's gonna be updated ever)
    // sound data
}__attribute__((packed));

FILE *nsFile = NULL;
struct nsfhead_s nsfHead;

byte bankswitch[8];
byte use_bankswitching = 0;
word bank_padding;

float nsf_playfreq;

byte cart_read_sequential(word addr);

typedef byte (*cart_read_t)(word addr);
cart_read_t cart_read = cart_read_sequential;

inline byte read_nsf_data(word offset)
{
    fseek(nsFile, offset+sizeof(struct nsfhead_s), SEEK_SET);
    int data = fgetc(nsFile);

    if(data == -1)
        return 0;

    return data;
}

byte cart_read_sequential(word addr)
{
    if(nsFile == NULL)
    {
        return 0;
    }

    return read_nsf_data(addr-nsfHead.load);
}

byte cart_read_bankswitch(word addr)
{
    // find which bank the address is located in, 0-7
    byte bank = (addr-0x8000)>>12;

    if(addr <= (0x8000|bank_padding))
            return 0;

    return read_nsf_data(bankswitch[bank]*0x1000 + (addr&0xFFF) - bank_padding );
}

M6502 cpu;

#ifdef DEBUG
    FILE *debugFile;
#endif

void Wr6502(register word Addr,register byte Value)
{
    if(Addr <= 0x7ff)
    {
        nes_wram[Addr] = Value;
    }
    else
    if(Addr <= 0x1fff)
    {
        nes_wram[Addr&0x7ff] = Value;
    }
    else
    if( (Addr >= 0x5ff8) && (Addr <= 0x5fff) )
    {
        bankswitch[(Addr&0xF)-8] = Value;
    }
    else
    if( (Addr >= 0x6000) && (Addr <= 0x7fff) )
    {
    #ifdef DEBUG
     //   fprintf(debugFile, "SRAM write    : $%04X set to $%02X\n", Addr, Value);
    #endif
        nes_sram[Addr-0x6000] = Value;
    }
    else
    {
        apu_write(Addr, Value);
    }
}

byte Rd6502(register word Addr)
{
    if(Addr <= 0x7ff)
    {
        return nes_wram[Addr];
    }
    else
    if(Addr <= 0x1fff)
    {
        return nes_wram[Addr&0x7ff];
    }
    else if(Addr >= 0x8000)
    {
        return cart_read(Addr);
    }
    else
    if( (Addr >= 0x5ff8) && (Addr <= 0x5fff) )
    {
        return bankswitch[(Addr&0xF)-8];
    }
    else
    if( (Addr >= 0x6000) && (Addr <= 0x7fff) )
    {
    #ifdef DEBUG
     //   fprintf(debugFile, "SRAM read: $%04X is $%02X\n", Addr, nes_sram[Addr-0x6000]);
        return nes_sram[Addr-0x6000];
    #else
        return nes_sram[Addr-0x6000];
    #endif
    }
    else
    return 0;
}

byte Loop6502(register M6502 *R)
{
    if(R->PC.W < 0x8000)    // out of code section
        return INT_QUIT;

    return INT_NONE;
}

byte Patch6502(register byte Op,register M6502 *R)
{
    return 0;
}

#define M_PUSH(Rg)	Wr6502(0x0100|R->S,Rg);R->S--
#define M_POP(Rg)	R->S++;Rg=Op6502(0x0100|R->S)
int Call6502(register M6502 *R, register word PC, register byte A, register byte X)
{
    int icycles;
    int cycles;
    R->A = A;
    R->X = X;
    R->Y = 0;
    R->P = 0;
    R->S = 255;
    R->PC.W = PC;

    M_PUSH(0);
    M_PUSH(0);
    cycles = 0;

    while(R->PC.W >2)
    {
        icycles = (1-Exec6502(R, 1));
        cycles += icycles;
    }

    return cycles;
}

void *audiobuffer;
int bufferlen;
snd_pcm_t *audiohandle;
int samplesPerPlay;
volatile int playing;
struct apu_s* apu;
void initNSF(byte song)
{
    samplesPerPlay = (((float)SAMPLE_RATE)/nsf_playfreq);
    bufferlen = samplesPerPlay*4;
    AudioConfig cfg = {SAMPLE_RATE, SAMPLE_BITS, 1, 0,bufferlen};
    audiobuffer = Audio_ALSA_open(&cfg, &audiohandle);

    memset(nes_wram, 0x00, 0x800);

    cart_read = cart_read_sequential;
    if(use_bankswitching == 1)
    {
        memcpy(bankswitch, nsfHead.bankswitch, 8);
        cart_read = cart_read_bankswitch;
        bank_padding = nsfHead.load & 0xfff;
    }

    byte X, A;

    if(BIT(nsfHead.palntsc,0) | BIT(nsfHead.palntsc, 1))
    {
        X = 1; // PAL
    }
    else
    {
        X = 0; // NTSC
    }

    apu = apu_create(SAMPLE_RATE, X);
    apu_setcontext(apu);
    apu_reset(0);

    A = song;

    cpu.Trap = nsfHead.init;
    Call6502(&cpu, nsfHead.init, A, X);
}

#ifndef DEBUG
#ifndef NO_AALIB
aa_context* context;
#endif
#endif
void *play_thread(void* param)
{
    initNSF((int)param);

    int j;
    int16_t* cur;
    while(playing)
    {
        cur = (int16_t*)audiobuffer;

        for(j = 0; j < bufferlen; ++j)
        {
            if(j%(samplesPerPlay) == 0)Call6502(&cpu, nsfHead.play, 0, 0);
            *cur = apu_output()>>16;
        #ifndef DEBUG
        #ifndef NO_AALIB
            aa_putpixel(context, ((aa_imgwidth(context)<<8)/bufferlen*j)>>8,  ((aa_imgheight(context)*((int)(0x7fff-(*cur))))>>16) - (aa_imgheight(context)/3), 127);
        #endif
        #endif
            ++cur;
        }


        Audio_ALSA_write(audiohandle, audiobuffer, bufferlen);

    #ifndef DEBUG
    #ifndef NO_AALIB
        aa_fastrender(context, 0, 0, aa_scrwidth(context), aa_scrheight(context));
        aa_flush(context);

        memset(aa_image(context), 0, aa_imgwidth(context)*aa_imgheight(context));
    #endif
    #endif
    }


    Audio_ALSA_close(audiohandle, audiobuffer);
    apu_destroy(NULL);

    return NULL;
}

#define VER_MAJ 0
#define VER_REV 1

void usage(void)
{
    fprintf(stderr,"Usage: tinynsf file.nsf\n");
}

void errorExit(int code)
{
    if(nsFile != NULL)
    {
        fclose(nsFile);
        nsFile = NULL;
    }

    exit(code);
}

int main(int argc, char **argv)
{
    int i;
    printf("TinyNSF v%i.%i\n", VER_MAJ, VER_REV);

    if(argc < 2)
    {
        fprintf(stderr, "Filename must be specified.\n");
        usage();
        errorExit(EXIT_FAILURE);
    }

    nsFile = fopen(argv[1], "rb");
    if(!nsFile)
    {
        fprintf(stderr, "Could not open specified file, \'%s\'.\n", argv[1]);
        errorExit(EXIT_FAILURE);
    }


    if(fread(&nsfHead, 1, sizeof(struct nsfhead_s), nsFile) < sizeof(struct nsfhead_s))
    {
        fprintf(stderr, "Error: invalid file, shorter than NSF header.\n");
        errorExit(EXIT_FAILURE);
    }

    if(memcmp(nsfHead.id, "NESM\x1A", 5) != 0)
    {
        fprintf(stderr, "Error: not a NSF file.\n");
        errorExit(EXIT_FAILURE);
    }

    if(nsfHead.version != 1)
    {
        fprintf(stderr, "Error: invalid NSF version.\n");
        errorExit(EXIT_FAILURE);
    }

    if( (nsfHead.songs == 0) || (nsfHead.start == 0) )
    {
        fprintf(stderr, "Error: no songs in NSF.\n");
        errorExit(EXIT_FAILURE);
    }

    use_bankswitching = 0;
    for(i = 0; i < 8; ++i)
    {
        if(nsfHead.bankswitch[i] != 0)
        {
            use_bankswitching = 1;
            break;
        }
    }

    printf ("Loaded a valid NSF.\n\n");
    printf ("\n");

    printf ("TITLE:\t\t%s\n", nsfHead.name);
    printf ("ARTIST:\t\t%s\n", nsfHead.artist);
    printf ("COPYRIGHT:\t%s\n", nsfHead.copyright);
    printf ("\n");

    printf ("Load:\t\t$%04X\n", nsfHead.load);
    printf ("Init:\t\t$%04X\n", nsfHead.init);
    printf ("Play:\t\t$%04X\n", nsfHead.play);
    printf ("\n");

    if(use_bankswitching == 1)
    {
        printf ("Tune uses bankswitching:\n");
        printf ("Banks:\t\t");
        for(i = 0; i < 8; ++i)
        {
            printf ("$%02X ", nsfHead.bankswitch[i]);
        }

        printf ("\n");
        printf ("\n");
    }

    printf ("Clock standard:\t");
    if(nsfHead.palntsc&1)
    {
        printf ("PAL\n");
        printf ("Play Freq: %f Hz\n", 1000000.0f / nsfHead.speedpal);
        nsf_playfreq = 1000000.0f / nsfHead.speedpal;
    }
    else
    if(nsfHead.palntsc>>1)
    {
        printf ("PAL & NTSC\n");
        printf ("\n");
        printf ("Play Freq PAL: %f Hz\n", 1000000.0f / nsfHead.speedpal);
        printf ("Play Freq NTSC: %f Hz\n", 1000000.0f / nsfHead.speedntsc);
        nsf_playfreq = 1000000.0f / nsfHead.speedpal;
    }
    else
    {
        printf ("NTSC\n");
        printf ("Play Freq: %f Hz\n", 1000000.0f / nsfHead.speedntsc);
        nsf_playfreq = 1000000.0f / nsfHead.speedntsc;
    }

    if(nsfHead.extsnd != 0)
    {
        printf ("Tune uses extra sound chip(s):\n");
        printf ("\t");
        if(BIT(nsfHead.extsnd, 0)) printf ("VRC6 ");
        if(BIT(nsfHead.extsnd, 1)) printf ("VRC7 ");
        if(BIT(nsfHead.extsnd, 2)) printf ("FDS ");
        if(BIT(nsfHead.extsnd, 3)) printf ("MMC5 ");
        if(BIT(nsfHead.extsnd, 4)) printf ("Namco_163 ");
        if(BIT(nsfHead.extsnd, 5)) printf ("Sunsoft_5B ");
        printf ("\n");
    }

#ifndef DEBUG
#ifndef NO_AALIB
    context = aa_autoinit(&aa_defparams);
    if(context == NULL) {
        fprintf(stderr,"Cannot initialize AA-lib. Sorry\n");
        errorExit(1);
    }
#endif
#endif

#ifdef DEBUG
    debugFile = fopen("tinynsf_debug.log", "w");
#endif

    // init
    int curSong = 1;
    pthread_t playThread = 0;

play_next:
    playing = 1;
    if(pthread_create(&playThread, NULL, play_thread, (void*)curSong-1) != 0)
    {
        fprintf(stderr, "Play thread creation unsuccessful.\n");
        errorExit(0);
    }

    printf("Song %i/%i\n", curSong, nsfHead.songs);
    printf("Playing... press return to play next song.\n");

#ifndef DEBUG
    getchar();
#else
    for(;;);
#endif

    playing = 0;
    pthread_join(playThread, NULL);

    if(curSong != nsfHead.songs)
    {
        ++curSong;
        goto play_next;
    }

    fclose(nsFile);

#ifdef DEBUG
    fclose(debugFile);
#endif

#ifndef DEBUG
#ifndef NO_AALIB
    aa_close(context);
#endif
#endif

    return 0;
}
