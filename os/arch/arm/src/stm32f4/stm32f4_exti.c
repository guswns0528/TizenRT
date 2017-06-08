/****************************************************************************
 * arch/arm/src/stm32f4/stm32f4_exti.c
 *
 *   Copyright (C) 2009, 2011-2012, 2015, 2017 Gregory Nutt. All rights reserved.
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
#include <tinyara/irq.h>
#include <tinyara/arch.h>

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <arch/irq.h>

#include "up_arch.h"
#include "chip.h"
#include "stm32f4_gpio.h"
#include "stm32f4_exti.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct gpio_callback_s
{
    xcpt_t callback;
    void  *arg;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Interrupt handlers attached to each EXTI */

static struct gpio_callback_s g_gpio_callbacks[16];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Interrupt Service Routines - Dispatchers
 ****************************************************************************/

#define ALL_ISRS \
EXTI_ISR(0) \
EXTI_ISR(1) \
EXTI_ISR(2) \
EXTI_ISR(3) \
EXTI_ISR(4) \
EXTI_ISR(5) \
EXTI_ISR(6) \
EXTI_ISR(7) \
EXTI_ISR(8) \
EXTI_ISR(9) \
EXTI_ISR(10) \
EXTI_ISR(11) \
EXTI_ISR(12) \
EXTI_ISR(13) \
EXTI_ISR(14) \
EXTI_ISR(15)

#define EXTI_ISR_NAME(__n) stm32f4_exti##__n##isr
#define EXTI_ISR(__n) \
static int EXTI_ISR_NAME(__n)(int irq, void *context, void *arg)    \
{                                                                   \
    int ret = OK;                                                   \
                                                                    \
    /* Clear the pending interrupt */                               \
                                                                    \
    putreg32(1 << __n, STM32_EXTI_PR);                              \
                                                                    \
    /* And dispatch the interrupt to the handler */                 \
                                                                    \
    if (g_gpio_callbacks[__n].callback != NULL)                     \
    {                                                               \
        xcpt_t callback = g_gpio_callbacks[__n].callback;           \
        void *cbarg = g_gpio_callbacks[__n].arg;                    \
                                                                    \
        ret = callback(irq, context, cbarg);                        \
    }                                                               \
    return ret;                                                     \
}

ALL_ISRS
#undef EXTI_ISR
#define EXTI_ISR(__n) EXTI_ISR_NAME(__n),
static xcpt_t exti_handlers[] = {
    ALL_ISRS
};
#undef STM32F4_EXTI_ISR
#undef ALL_ISRS

