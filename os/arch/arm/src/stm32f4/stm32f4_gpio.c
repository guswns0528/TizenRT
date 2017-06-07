/****************************************************************************
 * arch/arm/src/stm32f4/stm32f4_gpio.c
 *
 *   Copyright (C) 2009, 2011-2012 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2011 Uros Platise. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           Uros Platise <uros.platise@isotel.eu>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <tinyara/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <tinyara/irq.h>

#include "up_arch.h"

#include "chip.h"
#include "stm32f4_gpio.h"
#include "stm32f4_syscfg.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/
/* Base addresses for each GPIO block */

const uint32_t g_gpiobase[STM32_NGPIO_PORTS] =
{
#if STM32_NGPIO_PORTS > 0
    STM32_GPIOA_BASE,
#endif
#if STM32_NGPIO_PORTS > 1
    STM32_GPIOB_BASE,
#endif
#if STM32_NGPIO_PORTS > 2
    STM32_GPIOC_BASE,
#endif
#if STM32_NGPIO_PORTS > 3
    STM32_GPIOD_BASE,
#endif
#if STM32_NGPIO_PORTS > 4
    STM32_GPIOE_BASE,
#endif
#if STM32_NGPIO_PORTS > 5
    STM32_GPIOF_BASE,
#endif
#if STM32_NGPIO_PORTS > 6
    STM32_GPIOG_BASE,
#endif
#if STM32_NGPIO_PORTS > 7
    STM32_GPIOH_BASE,
#endif
#if STM32_NGPIO_PORTS > 8
    STM32_GPIOI_BASE,
#endif
#if STM32_NGPIO_PORTS > 9
    STM32_GPIOJ_BASE,
#endif
#if STM32_NGPIO_PORTS > 10
    STM32_GPIOK_BASE,
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/
/****************************************************************************
 * Public Functions
 ****************************************************************************/


