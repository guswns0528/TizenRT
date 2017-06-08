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

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32f4_gpiosetevent
 *
 * Description:
 *   Sets/clears GPIO based event and interrupt triggers.
 *
 * Input Parameters:
 *  - pinset:      GPIO pin configuration
 *  - risingedge:  Enables interrupt on rising edges
 *  - fallingedge: Enables interrupt on falling edges
 *  - event:       Generate event when set
 *  - func:        When non-NULL, generate interrupt
 *  - arg:         Argument passed to the interrupt callback
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure indicating the
 *   nature of the failure.
 *
 ****************************************************************************/

int stm32f4_gpiosetevent(uint32_t pinset, bool risingedge, bool fallingedge,
        bool event, xcpt_t func, void *arg)
{
    uint32_t pin = pinset & GPIO_PIN_MASK;
    uint32_t exti = STM32_EXTI_BIT(pin);
    int      irq;
    xcpt_t   handler;

    /* Select the interrupt handler for this EXTI pin */

    irq     = pin + STM32_IRQ_EXTI0;
    handler = exti_handlers[pin];

    /* Get the previous GPIO IRQ handler; Save the new IRQ handler. */

    g_gpio_callbacks[pin].callback = func;
    g_gpio_callbacks[pin].arg      = arg;

    /* Install external interrupt handlers */

    if (func)
    {
        irq_attach(irq, handler, NULL);
        up_enable_irq(irq);
    }
    else
    {
        up_disable_irq(irq);
    }

    /* Configure GPIO, enable EXTI line enabled if event or interrupt is
     * enabled.
     */

    if (event || func)
    {
        pinset |= GPIO_EXTI;
    }

    stm32f4_configgpio(pinset);

    /* Configure rising/falling edges */

    modifyreg32(STM32_EXTI_RTSR,
            risingedge ? 0 : exti,
            risingedge ? exti : 0);
    modifyreg32(STM32_EXTI_FTSR,
            fallingedge ? 0 : exti,
            fallingedge ? exti : 0);

    /* Enable Events and Interrupts */

    modifyreg32(STM32_EXTI_EMR,
            event ? 0 : exti,
            event ? exti : 0);
    modifyreg32(STM32_EXTI_IMR,
            func ? 0 : exti,
            func ? exti : 0);

    return OK;
}
