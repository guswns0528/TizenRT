#include <tinyara/config.h>
#include <stdint.h>

#include "up_internal.h"

extern void up_init_lowled(void);
extern void toggle_led(void);
extern void stm32_clockconfig(void);
extern void up_lowputc(char ch);

void halt_loop(void)
{
    while (1)
    {
        int i;
        for (i = 0; i < 16800000 / 7; i++)
        {
        }
        toggle_led();
    }
}

void arm_boot(void)
{
    const uint8_t *src;
    uint8_t *dest;

    stm32_clockconfig();
    up_init_lowled();


    for (dest = (uint8_t*)&_sbss; dest < (uint8_t*)&_ebss; )
    {
        *dest++ = 0;
    }

    for (src = (const uint8_t*)&_eronly, dest = (uint8_t*)&_sdata; dest < (uint8_t*)&_edata; )
    {
        *dest++ = *src++;
    }

    halt_loop();
}
