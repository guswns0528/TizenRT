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
}
