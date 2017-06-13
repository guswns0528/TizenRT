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
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_irqinitialize
 ****************************************************************************/

void up_irqinitialize(void)
{
    uint32_t regaddr;
    int num_priority_registers;
    int i;

    /* Disable all interrupts */

    for (i = 0; i < NR_IRQS - STM32_IRQ_FIRST; i += 32)
    {
        putreg32(0xffffffff, NVIC_IRQ_CLEAR(i));
    }

    /* The standard location for the vector table is at the beginning of FLASH
     * at address 0x0800:0000.  If we are using the STMicro DFU bootloader, then
     * the vector table will be offset to a different location in FLASH and we
     * will need to set the NVIC vector location to this alternative location.
     */

    putreg32((uint32_t)_vectors, NVIC_VECTAB);

#ifdef CONFIG_ARCH_RAMVECTORS
    /* If CONFIG_ARCH_RAMVECTORS is defined, then we are using a RAM-based
     * vector table that requires special initialization.
     */

    up_ramvec_initialize();
#endif

    /* Set all interrupts (and exceptions) to the default priority */

    putreg32(DEFPRIORITY32, NVIC_SYSH4_7_PRIORITY);
    putreg32(DEFPRIORITY32, NVIC_SYSH8_11_PRIORITY);
    putreg32(DEFPRIORITY32, NVIC_SYSH12_15_PRIORITY);

    /* The NVIC ICTR register (bits 0-4) holds the number of of interrupt
     * lines that the NVIC supports:
     *
     *  0 -> 32 interrupt lines,  8 priority registers
     *  1 -> 64 "       " "   ", 16 priority registers
     *  2 -> 96 "       " "   ", 32 priority registers
     *  ...
     */

    num_priority_registers = (getreg32(NVIC_ICTR) + 1) * 8;

    /* Now set all of the interrupt lines to the default priority */

    regaddr = NVIC_IRQ0_3_PRIORITY;
    while (num_priority_registers--)
    {
        putreg32(DEFPRIORITY32, regaddr);
        regaddr += 4;
    }

    /* currents_regs is non-NULL only while processing an interrupt */

    current_regs = NULL;

    /* Attach the SVCall and Hard Fault exception handlers.  The SVCall
     * exception is used for performing context switches; The Hard Fault
     * must also be caught because a SVCall may show up as a Hard Fault
     * under certain conditions.
     */

    irq_attach(STM32_IRQ_SVCALL, up_svcall, NULL);
    irq_attach(STM32_IRQ_HARDFAULT, up_hardfault, NULL);

    /* Set the priority of the SVCall interrupt */

#ifdef CONFIG_ARCH_IRQPRIO
    /* up_prioritize_irq(STM32_IRQ_PENDSV, NVIC_SYSH_PRIORITY_MIN); */
#endif
#ifdef CONFIG_ARMV7M_USEBASEPRI
    stm32_prioritize_syscall(NVIC_SYSH_SVCALL_PRIORITY);
#endif

    /* If the MPU is enabled, then attach and enable the Memory Management
     * Fault handler.
     */

#ifdef CONFIG_ARM_MPU
    irq_attach(STM32_IRQ_MEMFAULT, up_memfault, NULL);
    up_enable_irq(STM32_IRQ_MEMFAULT);
#endif

    /* Attach all other processor exceptions (except reset and sys tick) */

#ifdef CONFIG_DEBUG_FEATURES
    irq_attach(STM32_IRQ_NMI, stm32_nmi, NULL);
#ifndef CONFIG_ARM_MPU
    irq_attach(STM32_IRQ_MEMFAULT, up_memfault, NULL);
#endif
    irq_attach(STM32_IRQ_BUSFAULT, stm32_busfault, NULL);
    irq_attach(STM32_IRQ_USAGEFAULT, stm32_usagefault, NULL);
    irq_attach(STM32_IRQ_PENDSV, stm32_pendsv, NULL);
    irq_attach(STM32_IRQ_DBGMONITOR, stm32_dbgmonitor, NULL);
    irq_attach(STM32_IRQ_RESERVED, stm32_reserved, NULL);
#endif

    stm32_dumpnvic("initial", NR_IRQS);

#ifndef CONFIG_SUPPRESS_INTERRUPTS

    /* And finally, enable interrupts */

    irqenable();
#endif
}

/****************************************************************************
 * Name: up_disable_irq
 *
 * Description:
 *   Disable the IRQ specified by 'irq'
 *
 ****************************************************************************/

void up_disable_irq(int irq)
{
    uintptr_t regaddr;
    uint32_t regval;
    uint32_t bit;

    if (stm32_irqinfo(irq, &regaddr, &bit, NVIC_CLRENA_OFFSET) == 0)
    {
        /* Modify the appropriate bit in the register to disable the interrupt.
         * For normal interrupts, we need to set the bit in the associated
         * Interrupt Clear Enable register.  For other exceptions, we need to
         * clear the bit in the System Handler Control and State Register.
         */

        if (irq >= STM32_IRQ_FIRST)
        {
            putreg32(bit, regaddr);
        }
        else
        {
            regval  = getreg32(regaddr);
            regval &= ~bit;
            putreg32(regval, regaddr);
        }
    }

    // stm32_dumpnvic("disable", irq);
}

/****************************************************************************
 * Name: up_enable_irq
 *
 * Description:
 *   Enable the IRQ specified by 'irq'
 *
 ****************************************************************************/

void up_enable_irq(int irq)
{
    uintptr_t regaddr;
    uint32_t regval;
    uint32_t bit;

    if (stm32_irqinfo(irq, &regaddr, &bit, NVIC_ENA_OFFSET) == 0)
    {
        /* Modify the appropriate bit in the register to enable the interrupt.
         * For normal interrupts, we need to set the bit in the associated
         * Interrupt Set Enable register.  For other exceptions, we need to
         * set the bit in the System Handler Control and State Register.
         */

        if (irq >= STM32_IRQ_FIRST)
        {
            putreg32(bit, regaddr);
        }
        else
        {
            regval  = getreg32(regaddr);
            regval |= bit;
            putreg32(regval, regaddr);
        }
    }

    // stm32_dumpnvic("enable", irq);
}

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

