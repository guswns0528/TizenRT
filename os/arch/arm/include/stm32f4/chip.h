/****************************************************************************
 *
 * Copyright 2016 Samsung Electronics All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 * either express or implied. See the License for the specific
 * language governing permissions and limitations under the License.
 *
 ****************************************************************************/
/****************************************************************************************************
 * arch/arm/include/stm32f4/chip.h
 *
 *   Copyright (C) 2013-2015 Gregory Nutt. All rights reserved.
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
 ****************************************************************************************************/

#ifndef __ARCH_ARM_INCLUDE_STM32F4_CHIP_H
#define __ARCH_ARM_INCLUDE_STM32F4_CHIP_H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <tinyara/config.h>

/****************************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************************/

#define STM32_NTIMERS         0	/* Four 16/32-bit timers */
#define STM32_NWIDETIMERS     0	/* No 32/64-bit timers */
#define STM32_NWDT            0	/* One watchdog timer */
#define STM32_NETHCONTROLLERS 0	/* One Ethernet controller */
#define STM32_NLCD            0	/* No LCD controller */
#undef  STM32_ETHTS				/* No timestamp register */
#define STM32_NSSI            0	/* Two SSI modules */
#define STM32_NUARTS          1	/* Two UART modules */
#define STM32_NI2C            0	/* Two I2C modules */
#define STM32_NADC            0	/* One ADC module */
#define STM32_NPWM            0	/* No PWM generator modules */
#define STM32_NQEI            0	/* No quadrature encoders */
#define STM32_NPORTS          0	/* 8 Ports (GPIOA-H) 5-38 GPIOs */
#define STM32_NCANCONTROLLER  0	/* No CAN controllers */
#define STM32_NUSBOTGFS       0	/* No USB 2.0 OTG FS */
#define STM32_NUSBOTGHS       0	/* No USB 2.0 OTG HS */
#define STM32_NCRC            0	/* No CRC module */
#define STM32_NAES            0	/* No AES module */
#define STM32_NDES            0	/* No DES module */
#define STM32_NHASH           0	/* No SHA1/MD5 hash module */

#  undef  CONFIG_STM32_STM32L15XX            /* STM32L151xx and STM32L152xx family */
#  undef  CONFIG_STM32_ENERGYLITE            /* STM32L EnergyLite family */
#  undef  CONFIG_STM32_STM32F10XX            /* STM32F10xxx family */
#  undef  CONFIG_STM32_LOWDENSITY            /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 16/32 Kbytes */
#  undef  CONFIG_STM32_MEDIUMDENSITY         /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 64/128 Kbytes */
#  undef  CONFIG_STM32_MEDIUMPLUSDENSITY     /* STM32L15xxC w/ 32/256 Kbytes */
#  undef  CONFIG_STM32_HIGHDENSITY           /* STM32F100x, STM32F101x, and STM32F103x w/ 256/512 Kbytes */
#  undef  CONFIG_STM32_VALUELINE             /* STM32F100x */
#  undef  CONFIG_STM32_CONNECTIVITYLINE      /* STM32F105x and STM32F107x */
#  undef  CONFIG_STM32_STM32F20XX            /* STM32F205x and STM32F207x */
#  undef  CONFIG_STM32_STM32F30XX            /* STM32F30xxx family */
#  undef  CONFIG_STM32_STM32F33XX            /* STM32F33xxx family */
#  undef  CONFIG_STM32_STM32F37XX            /* STM32F37xxx family */
#  define CONFIG_STM32_STM32F40XX        1   /* STM32F405xx, STM32407xx and STM32F427/437 */
#  define STM32_NFSMC                    1   /* FSMC */
#  define STM32_NATIM                    2   /* Two advanced timers TIM1 and 8 */
#  define STM32_NGTIM                    4   /* 16-bit general timers TIM3 and 4 with DMA
                                              * 32-bit general timers TIM2 and 5 with DMA */
#  define STM32_NGTIMNDMA                6   /* 16-bit general timers TIM9-14 without DMA */
#  define STM32_NBTIM                    2   /* Two basic timers, TIM6-7 */
#  define STM32_NDMA                     2   /* DMA1-2 */
#  define STM32_NSPI                     4   /* SPI1-4 */
#  define STM32_NI2S                     2   /* I2S1-2 (multiplexed with SPI2-3) */
#  define STM32_NUSART                   8   /* USART1-3 and 6, UART 4-5 and 7-8 */
#  define STM32_NI2C                     3   /* I2C1-3 */
#  define STM32_NCAN                     2   /* CAN1-2 */
#  define STM32_NSDIO                    1   /* SDIO */
#  define STM32_NLCD                     0   /* No LCD */
#  define STM32_NUSBOTG                  1   /* USB OTG FS/HS */
#  define STM32_NGPIO                    139 /* GPIOA-I */
#  define STM32_NADC                     3   /* 12-bit ADC1-3, 24 channels */
#  define STM32_NDAC                     2   /* 12-bit DAC1-2 */
#  define STM32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                1   /* 100/100 Ethernet MAC */
#  define STM32_NRNG                     1   /* Random number generator (RNG) */
#  define STM32_NDCMI                    1   /* Digital camera interface (DCMI) */

/* NVIC priority levels *************************************************************/

#define NVIC_SYSH_PRIORITY_MIN     0xf0 /* All bits set in minimum priority */
#define NVIC_SYSH_PRIORITY_DEFAULT 0x80 /* Midpoint is the default */
#define NVIC_SYSH_PRIORITY_MAX     0x00 /* Zero is maximum priority */
#define NVIC_SYSH_PRIORITY_STEP    0x10 /* Four bits of interrupt priority used */

/* If CONFIG_ARMV7M_USEBASEPRI is selected, then interrupts will be disabled
 * by setting the BASEPRI register to NVIC_SYSH_DISABLE_PRIORITY so that most
 * interrupts will not have execution priority.  SVCall must have execution
 * priority in all cases.
 *
 * In the normal cases, interrupts are not nest-able and all interrupts run
 * at an execution priority between NVIC_SYSH_PRIORITY_MIN and
 * NVIC_SYSH_PRIORITY_MAX (with NVIC_SYSH_PRIORITY_MAX reserved for SVCall).
 *
 * If, in addition, CONFIG_ARCH_HIPRI_INTERRUPT is defined, then special
 * high priority interrupts are supported.  These are not "nested" in the
 * normal sense of the word.  These high priority interrupts can interrupt
 * normal processing but execute outside of OS (although they can "get back
 * into the game" via a PendSV interrupt).
 *
 * In the normal course of things, interrupts must occasionally be disabled
 * using the up_irq_save() inline function to prevent contention in use of
 * resources that may be shared between interrupt level and non-interrupt
 * level logic.  Now the question arises, if CONFIG_ARCH_HIPRI_INTERRUPT,
 * do we disable all interrupts (except SVCall), or do we only disable the
 * "normal" interrupts.  Since the high priority interrupts cannot interact
 * with the OS, you may want to permit the high priority interrupts even if
 * interrupts are disabled.  The setting CONFIG_ARCH_INT_DISABLEALL can be
 * used to select either behavior:
 *
 *   ----------------------------+--------------+----------------------------
 *   CONFIG_ARCH_HIPRI_INTERRUPT |      NO      |             YES
 *   ----------------------------+--------------+--------------+-------------
 *   CONFIG_ARCH_INT_DISABLEALL  |     N/A      |     YES      |      NO
 *   ----------------------------+--------------+--------------+-------------
 *                               |              |              |    SVCall
 *                               |    SVCall    |    SVCall    |    HIGH
 *   Disable here and below --------> MAXNORMAL ---> HIGH --------> MAXNORMAL
 *                               |              |    MAXNORMAL |
 *   ----------------------------+--------------+--------------+-------------
 */

#if defined(CONFIG_ARCH_HIPRI_INTERRUPT) && defined(CONFIG_ARCH_INT_DISABLEALL)
#  define NVIC_SYSH_MAXNORMAL_PRIORITY  (NVIC_SYSH_PRIORITY_MAX + 2*NVIC_SYSH_PRIORITY_STEP)
#  define NVIC_SYSH_HIGH_PRIORITY       (NVIC_SYSH_PRIORITY_MAX + NVIC_SYSH_PRIORITY_STEP)
#  define NVIC_SYSH_DISABLE_PRIORITY    NVIC_SYSH_HIGH_PRIORITY
#  define NVIC_SYSH_SVCALL_PRIORITY     NVIC_SYSH_PRIORITY_MAX
#else
#  define NVIC_SYSH_MAXNORMAL_PRIORITY  (NVIC_SYSH_PRIORITY_MAX + NVIC_SYSH_PRIORITY_STEP)
#  define NVIC_SYSH_HIGH_PRIORITY       NVIC_SYSH_PRIORITY_MAX
#  define NVIC_SYSH_DISABLE_PRIORITY    NVIC_SYSH_MAXNORMAL_PRIORITY
#  define NVIC_SYSH_SVCALL_PRIORITY     NVIC_SYSH_PRIORITY_MAX
#endif


/****************************************************************************************************
 * Public Types
 ****************************************************************************************************/

/****************************************************************************************************
 * Public Data
 ****************************************************************************************************/

/****************************************************************************************************
 * Public Functions
 ****************************************************************************************************/

#endif							/* __ARCH_ARM_INCLUDE_STM32F4_CHIP_H */
