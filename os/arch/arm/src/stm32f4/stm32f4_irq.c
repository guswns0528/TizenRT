/****************************************************************************
 * arch/arm/src/stm32f4/stm32f4_irq.c
 *
 *   Copyright (C) 2009-2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#include <stdint.h>
#include <debug.h>

#include <tinyara/irq.h>
#include <tinyara/arch.h>
#include <arch/irq.h>

#include "nvic.h"
#include "up_arch.h"
#include "up_internal.h"

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Get a 32-bit version of the default priority */

#define DEFPRIORITY32 \
    (NVIC_SYSH_PRIORITY_DEFAULT << 24 | \
     NVIC_SYSH_PRIORITY_DEFAULT << 16 | \
     NVIC_SYSH_PRIORITY_DEFAULT << 8  | \
     NVIC_SYSH_PRIORITY_DEFAULT)

/* Given the address of a NVIC ENABLE register, this is the offset to
 * the corresponding CLEAR ENABLE register.
 */

#define NVIC_ENA_OFFSET    (0)
#define NVIC_CLRENA_OFFSET (NVIC_IRQ0_31_CLEAR - NVIC_IRQ0_31_ENABLE)


/****************************************************************************
 * Public Data
 ****************************************************************************/
volatile uint32_t *current_regs;

extern uint32_t _vectors[];

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_irqinfo
 *
 * Description:
 *   Given an IRQ number, provide the register and bit setting to enable or
 *   disable the irq.
 *
 ****************************************************************************/

static int stm32_irqinfo(int irq, uintptr_t *regaddr, uint32_t *bit,
        uintptr_t offset)
{
    int n;

    DEBUGASSERT(irq >= STM32_IRQ_NMI && irq < NR_IRQS);

    /* Check for external interrupt */

    if (irq >= STM32_IRQ_FIRST)
    {
        n        = irq - STM32_IRQ_FIRST;
        *regaddr = NVIC_IRQ_ENABLE(n) + offset;
        *bit     = (uint32_t)1 << (n & 0x1f);
    }

    /* Handle processor exceptions.  Only a few can be disabled */

    else
    {
        *regaddr = NVIC_SYSHCON;
        if (irq == STM32_IRQ_MEMFAULT)
        {
            *bit = NVIC_SYSHCON_MEMFAULTENA;
        }
        else if (irq == STM32_IRQ_BUSFAULT)
        {
            *bit = NVIC_SYSHCON_BUSFAULTENA;
        }
        else if (irq == STM32_IRQ_USAGEFAULT)
        {
            *bit = NVIC_SYSHCON_USGFAULTENA;
        }
        else if (irq == STM32_IRQ_SYSTICK)
        {
            *regaddr = NVIC_SYSTICK_CTRL;
            *bit = NVIC_SYSTICK_CTRL_ENABLE;
        }
        else
        {
            return ERROR; /* Invalid or unsupported exception */
        }
    }

    return OK;
}

/****************************************************************************
 * Name: stm32_dumpnvic
 *
 * Description:
 *   Dump some interesting NVIC registers
 *
 ****************************************************************************/

#if defined(CONFIG_DEBUG_IRQ_INFO)
static void stm32_dumpnvic(const char *msg, int irq)
{
    irqstate_t flags;

    flags = irqsave();

    irqinfo("NVIC (%s, irq=%d):\n", msg, irq);
    irqinfo("  INTCTRL:    %08x VECTAB:  %08x\n",
            getreg32(NVIC_INTCTRL), getreg32(NVIC_VECTAB));
#if 0
    irqinfo("  SYSH ENABLE MEMFAULT: %08x BUSFAULT: %08x USGFAULT: %08x SYSTICK: %08x\n",
            getreg32(NVIC_SYSHCON_MEMFAULTENA), getreg32(NVIC_SYSHCON_BUSFAULTENA),
            getreg32(NVIC_SYSHCON_USGFAULTENA), getreg32(NVIC_SYSTICK_CTRL_ENABLE));
#endif
    irqinfo("  IRQ ENABLE: %08x %08x %08x\n",
            getreg32(NVIC_IRQ0_31_ENABLE), getreg32(NVIC_IRQ32_63_ENABLE),
            getreg32(NVIC_IRQ64_95_ENABLE));
    irqinfo("  SYSH_PRIO:  %08x %08x %08x\n",
            getreg32(NVIC_SYSH4_7_PRIORITY), getreg32(NVIC_SYSH8_11_PRIORITY),
            getreg32(NVIC_SYSH12_15_PRIORITY));
    irqinfo("  IRQ PRIO:   %08x %08x %08x %08x\n",
            getreg32(NVIC_IRQ0_3_PRIORITY), getreg32(NVIC_IRQ4_7_PRIORITY),
            getreg32(NVIC_IRQ8_11_PRIORITY), getreg32(NVIC_IRQ12_15_PRIORITY));
    irqinfo("              %08x %08x %08x %08x\n",
            getreg32(NVIC_IRQ16_19_PRIORITY), getreg32(NVIC_IRQ20_23_PRIORITY),
            getreg32(NVIC_IRQ24_27_PRIORITY), getreg32(NVIC_IRQ28_31_PRIORITY));
    irqinfo("              %08x %08x %08x %08x\n",
            getreg32(NVIC_IRQ32_35_PRIORITY), getreg32(NVIC_IRQ36_39_PRIORITY),
            getreg32(NVIC_IRQ40_43_PRIORITY), getreg32(NVIC_IRQ44_47_PRIORITY));
    irqinfo("              %08x %08x %08x %08x\n",
            getreg32(NVIC_IRQ48_51_PRIORITY), getreg32(NVIC_IRQ52_55_PRIORITY),
            getreg32(NVIC_IRQ56_59_PRIORITY), getreg32(NVIC_IRQ60_63_PRIORITY));
    irqinfo("              %08x\n",
            getreg32(NVIC_IRQ64_67_PRIORITY));

    irqrestore(flags);
}
#else
#  define stm32_dumpnvic(msg, irq)
#endif



/****************************************************************************
 * Name: up_ack_irq
 *
 * Description:
 *   Acknowledge the IRQ
 *
 ****************************************************************************/

void up_ack_irq(int irq)
{
}

