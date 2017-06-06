/************************************************************************************
 * arch/arm/src/stm32f4/stm32f4_gpio.h
 *
 *   Copyright (C) 2009, 2011-2012, 2015 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32F4_STM32F4_GPIO_H
#define __ARCH_ARM_SRC_STM32F4_STM32F4_GPIO_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <tinyara/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#  include <stdbool.h>
#endif

#include <tinyara/irq.h>

#include "chip.h"

/************************************************************************************
 * Pre-Processor Declarations
 ************************************************************************************/

#define STM32_NGPIO_PORTS          ((STM32_NGPIO + 15) >> 4)

/* Register Offsets *****************************************************************/

#define STM32_GPIO_MODER_OFFSET    0x0000 /* GPIO port mode register */
#define STM32_GPIO_OTYPER_OFFSET   0x0004 /* GPIO port output type register */
#define STM32_GPIO_OSPEED_OFFSET   0x0008 /* GPIO port output speed register */
#define STM32_GPIO_PUPDR_OFFSET    0x000c /* GPIO port pull-up/pull-down register */
#define STM32_GPIO_IDR_OFFSET      0x0010 /* GPIO port input data register */
#define STM32_GPIO_ODR_OFFSET      0x0014 /* GPIO port output data register */
#define STM32_GPIO_BSRR_OFFSET     0x0018 /* GPIO port bit set/reset register */
#define STM32_GPIO_LCKR_OFFSET     0x001c /* GPIO port configuration lock register */
#define STM32_GPIO_AFRL_OFFSET     0x0020 /* GPIO alternate function low register */
#define STM32_GPIO_AFRH_OFFSET     0x0024 /* GPIO alternate function high register */

/* Register Addresses ***************************************************************/

#if STM32_NGPIO_PORTS > 0
#  define STM32_GPIOA_MODER        (STM32_GPIOA_BASE+STM32_GPIO_MODER_OFFSET)
#  define STM32_GPIOA_OTYPER       (STM32_GPIOA_BASE+STM32_GPIO_OTYPER_OFFSET)
#  define STM32_GPIOA_OSPEED       (STM32_GPIOA_BASE+STM32_GPIO_OSPEED_OFFSET)
#  define STM32_GPIOA_PUPDR        (STM32_GPIOA_BASE+STM32_GPIO_PUPDR_OFFSET)
#  define STM32_GPIOA_IDR          (STM32_GPIOA_BASE+STM32_GPIO_IDR_OFFSET)
#  define STM32_GPIOA_ODR          (STM32_GPIOA_BASE+STM32_GPIO_ODR_OFFSET)
#  define STM32_GPIOA_BSRR         (STM32_GPIOA_BASE+STM32_GPIO_BSRR_OFFSET)
#  define STM32_GPIOA_LCKR         (STM32_GPIOA_BASE+STM32_GPIO_LCKR_OFFSET)
#  define STM32_GPIOA_AFRL         (STM32_GPIOA_BASE+STM32_GPIO_AFRL_OFFSET)
#  define STM32_GPIOA_AFRH         (STM32_GPIOA_BASE+STM32_GPIO_AFRH_OFFSET)
#endif

#if STM32_NGPIO_PORTS > 1
#  define STM32_GPIOB_MODER        (STM32_GPIOB_BASE+STM32_GPIO_MODER_OFFSET)
#  define STM32_GPIOB_OTYPER       (STM32_GPIOB_BASE+STM32_GPIO_OTYPER_OFFSET)
#  define STM32_GPIOB_OSPEED       (STM32_GPIOB_BASE+STM32_GPIO_OSPEED_OFFSET)
#  define STM32_GPIOB_PUPDR        (STM32_GPIOB_BASE+STM32_GPIO_PUPDR_OFFSET)
#  define STM32_GPIOB_IDR          (STM32_GPIOB_BASE+STM32_GPIO_IDR_OFFSET)
#  define STM32_GPIOB_ODR          (STM32_GPIOB_BASE+STM32_GPIO_ODR_OFFSET)
#  define STM32_GPIOB_BSRR         (STM32_GPIOB_BASE+STM32_GPIO_BSRR_OFFSET)
#  define STM32_GPIOB_LCKR         (STM32_GPIOB_BASE+STM32_GPIO_LCKR_OFFSET)
#  define STM32_GPIOB_AFRL         (STM32_GPIOB_BASE+STM32_GPIO_AFRL_OFFSET)
#  define STM32_GPIOB_AFRH         (STM32_GPIOB_BASE+STM32_GPIO_AFRH_OFFSET)
#endif

#if STM32_NGPIO_PORTS > 2
#  define STM32_GPIOC_MODER        (STM32_GPIOC_BASE+STM32_GPIO_MODER_OFFSET)
#  define STM32_GPIOC_OTYPER       (STM32_GPIOC_BASE+STM32_GPIO_OTYPER_OFFSET)
#  define STM32_GPIOC_OSPEED       (STM32_GPIOC_BASE+STM32_GPIO_OSPEED_OFFSET)
#  define STM32_GPIOC_PUPDR        (STM32_GPIOC_BASE+STM32_GPIO_PUPDR_OFFSET)
#  define STM32_GPIOC_IDR          (STM32_GPIOC_BASE+STM32_GPIO_IDR_OFFSET)
#  define STM32_GPIOC_ODR          (STM32_GPIOC_BASE+STM32_GPIO_ODR_OFFSET)
#  define STM32_GPIOC_BSRR         (STM32_GPIOC_BASE+STM32_GPIO_BSRR_OFFSET)
#  define STM32_GPIOC_LCKR         (STM32_GPIOC_BASE+STM32_GPIO_LCKR_OFFSET)
#  define STM32_GPIOC_AFRL         (STM32_GPIOC_BASE+STM32_GPIO_AFRL_OFFSET)
#  define STM32_GPIOC_AFRH         (STM32_GPIOC_BASE+STM32_GPIO_AFRH_OFFSET)
#endif

#if STM32_NGPIO_PORTS > 3
#  define STM32_GPIOD_MODER        (STM32_GPIOD_BASE+STM32_GPIO_MODER_OFFSET)
#  define STM32_GPIOD_OTYPER       (STM32_GPIOD_BASE+STM32_GPIO_OTYPER_OFFSET)
#  define STM32_GPIOD_OSPEED       (STM32_GPIOD_BASE+STM32_GPIO_OSPEED_OFFSET)
#  define STM32_GPIOD_PUPDR        (STM32_GPIOD_BASE+STM32_GPIO_PUPDR_OFFSET)
#  define STM32_GPIOD_IDR          (STM32_GPIOD_BASE+STM32_GPIO_IDR_OFFSET)
#  define STM32_GPIOD_ODR          (STM32_GPIOD_BASE+STM32_GPIO_ODR_OFFSET)
#  define STM32_GPIOD_BSRR         (STM32_GPIOD_BASE+STM32_GPIO_BSRR_OFFSET)
#  define STM32_GPIOD_LCKR         (STM32_GPIOD_BASE+STM32_GPIO_LCKR_OFFSET)
#  define STM32_GPIOD_AFRL         (STM32_GPIOD_BASE+STM32_GPIO_AFRL_OFFSET)
#  define STM32_GPIOD_AFRH         (STM32_GPIOD_BASE+STM32_GPIO_AFRH_OFFSET)
#endif

#if STM32_NGPIO_PORTS > 4
#  define STM32_GPIOE_MODER        (STM32_GPIOE_BASE+STM32_GPIO_MODER_OFFSET)
#  define STM32_GPIOE_OTYPER       (STM32_GPIOE_BASE+STM32_GPIO_OTYPER_OFFSET)
#  define STM32_GPIOE_OSPEED       (STM32_GPIOE_BASE+STM32_GPIO_OSPEED_OFFSET)
#  define STM32_GPIOE_PUPDR        (STM32_GPIOE_BASE+STM32_GPIO_PUPDR_OFFSET)
#  define STM32_GPIOE_IDR          (STM32_GPIOE_BASE+STM32_GPIO_IDR_OFFSET)
#  define STM32_GPIOE_ODR          (STM32_GPIOE_BASE+STM32_GPIO_ODR_OFFSET)
#  define STM32_GPIOE_BSRR         (STM32_GPIOE_BASE+STM32_GPIO_BSRR_OFFSET)
#  define STM32_GPIOE_LCKR         (STM32_GPIOE_BASE+STM32_GPIO_LCKR_OFFSET)
#  define STM32_GPIOE_AFRL         (STM32_GPIOE_BASE+STM32_GPIO_AFRL_OFFSET)
#  define STM32_GPIOE_AFRH         (STM32_GPIOE_BASE+STM32_GPIO_AFRH_OFFSET)
#endif

#if STM32_NGPIO_PORTS > 5
#  define STM32_GPIOF_MODER        (STM32_GPIOF_BASE+STM32_GPIO_MODER_OFFSET)
#  define STM32_GPIOF_OTYPER       (STM32_GPIOF_BASE+STM32_GPIO_OTYPER_OFFSET)
#  define STM32_GPIOF_OSPEED       (STM32_GPIOF_BASE+STM32_GPIO_OSPEED_OFFSET)
#  define STM32_GPIOF_PUPDR        (STM32_GPIOF_BASE+STM32_GPIO_PUPDR_OFFSET)
#  define STM32_GPIOF_IDR          (STM32_GPIOF_BASE+STM32_GPIO_IDR_OFFSET)
#  define STM32_GPIOF_ODR          (STM32_GPIOF_BASE+STM32_GPIO_ODR_OFFSET)
#  define STM32_GPIOF_BSRR         (STM32_GPIOF_BASE+STM32_GPIO_BSRR_OFFSET)
#  define STM32_GPIOF_LCKR         (STM32_GPIOF_BASE+STM32_GPIO_LCKR_OFFSET)
#  define STM32_GPIOF_AFRL         (STM32_GPIOF_BASE+STM32_GPIO_AFRL_OFFSET)
#  define STM32_GPIOF_AFRH         (STM32_GPIOF_BASE+STM32_GPIO_AFRH_OFFSET)
#endif

#if STM32_NGPIO_PORTS > 6
#  define STM32_GPIOG_MODER        (STM32_GPIOG_BASE+STM32_GPIO_MODER_OFFSET)
#  define STM32_GPIOG_OTYPER       (STM32_GPIOG_BASE+STM32_GPIO_OTYPER_OFFSET)
#  define STM32_GPIOG_OSPEED       (STM32_GPIOG_BASE+STM32_GPIO_OSPEED_OFFSET)
#  define STM32_GPIOG_PUPDR        (STM32_GPIOG_BASE+STM32_GPIO_PUPDR_OFFSET)
#  define STM32_GPIOG_IDR          (STM32_GPIOG_BASE+STM32_GPIO_IDR_OFFSET)
#  define STM32_GPIOG_ODR          (STM32_GPIOG_BASE+STM32_GPIO_ODR_OFFSET)
#  define STM32_GPIOG_BSRR         (STM32_GPIOG_BASE+STM32_GPIO_BSRR_OFFSET)
#  define STM32_GPIOG_LCKR         (STM32_GPIOG_BASE+STM32_GPIO_LCKR_OFFSET)
#  define STM32_GPIOG_AFRL         (STM32_GPIOG_BASE+STM32_GPIO_AFRL_OFFSET)
#  define STM32_GPIOG_AFRH         (STM32_GPIOG_BASE+STM32_GPIO_AFRH_OFFSET)
#endif

#if STM32_NGPIO_PORTS > 7
#  define STM32_GPIOH_MODER        (STM32_GPIOH_BASE+STM32_GPIO_MODER_OFFSET)
#  define STM32_GPIOH_OTYPER       (STM32_GPIOH_BASE+STM32_GPIO_OTYPER_OFFSET)
#  define STM32_GPIOH_OSPEED       (STM32_GPIOH_BASE+STM32_GPIO_OSPEED_OFFSET)
#  define STM32_GPIOH_PUPDR        (STM32_GPIOH_BASE+STM32_GPIO_PUPDR_OFFSET)
#  define STM32_GPIOH_IDR          (STM32_GPIOH_BASE+STM32_GPIO_IDR_OFFSET)
#  define STM32_GPIOH_ODR          (STM32_GPIOH_BASE+STM32_GPIO_ODR_OFFSET)
#  define STM32_GPIOH_BSRR         (STM32_GPIOH_BASE+STM32_GPIO_BSRR_OFFSET)
#  define STM32_GPIOH_LCKR         (STM32_GPIOH_BASE+STM32_GPIO_LCKR_OFFSET)
#  define STM32_GPIOH_AFRL         (STM32_GPIOH_BASE+STM32_GPIO_AFRL_OFFSET)
#  define STM32_GPIOH_AFRH         (STM32_GPIOH_BASE+STM32_GPIO_AFRH_OFFSET)
#endif

#if STM32_NGPIO_PORTS > 8
#  define STM32_GPIOI_MODER        (STM32_GPIOI_BASE+STM32_GPIO_MODER_OFFSET)
#  define STM32_GPIOI_OTYPER       (STM32_GPIOI_BASE+STM32_GPIO_OTYPER_OFFSET)
#  define STM32_GPIOI_OSPEED       (STM32_GPIOI_BASE+STM32_GPIO_OSPEED_OFFSET)
#  define STM32_GPIOI_PUPDR        (STM32_GPIOI_BASE+STM32_GPIO_PUPDR_OFFSET)
#  define STM32_GPIOI_IDR          (STM32_GPIOI_BASE+STM32_GPIO_IDR_OFFSET)
#  define STM32_GPIOI_ODR          (STM32_GPIOI_BASE+STM32_GPIO_ODR_OFFSET)
#  define STM32_GPIOI_BSRR         (STM32_GPIOI_BASE+STM32_GPIO_BSRR_OFFSET)
#  define STM32_GPIOI_LCKR         (STM32_GPIOI_BASE+STM32_GPIO_LCKR_OFFSET)
#  define STM32_GPIOI_AFRL         (STM32_GPIOI_BASE+STM32_GPIO_AFRL_OFFSET)
#  define STM32_GPIOI_AFRH         (STM32_GPIOI_BASE+STM32_GPIO_AFRH_OFFSET)
#endif

#if STM32_NGPIO_PORTS > 9
#  define STM32_GPIOJ_MODER        (STM32_GPIOJ_BASE+STM32_GPIO_MODER_OFFSET)
#  define STM32_GPIOJ_OTYPER       (STM32_GPIOJ_BASE+STM32_GPIO_OTYPER_OFFSET)
#  define STM32_GPIOJ_OSPEED       (STM32_GPIOJ_BASE+STM32_GPIO_OSPEED_OFFSET)
#  define STM32_GPIOJ_PUPDR        (STM32_GPIOJ_BASE+STM32_GPIO_PUPDR_OFFSET)
#  define STM32_GPIOJ_IDR          (STM32_GPIOJ_BASE+STM32_GPIO_IDR_OFFSET)
#  define STM32_GPIOJ_ODR          (STM32_GPIOJ_BASE+STM32_GPIO_ODR_OFFSET)
#  define STM32_GPIOJ_BSRR         (STM32_GPIOJ_BASE+STM32_GPIO_BSRR_OFFSET)
#  define STM32_GPIOJ_LCKR         (STM32_GPIOJ_BASE+STM32_GPIO_LCKR_OFFSET)
#  define STM32_GPIOJ_AFRL         (STM32_GPIOJ_BASE+STM32_GPIO_AFRL_OFFSET)
#  define STM32_GPIOJ_AFRH         (STM32_GPIOJ_BASE+STM32_GPIO_AFRH_OFFSET)
#endif

#if STM32_NGPIO_PORTS > 10
#  define STM32_GPIOK_MODER        (STM32_GPIOK_BASE+STM32_GPIO_MODER_OFFSET)
#  define STM32_GPIOK_OTYPER       (STM32_GPIOK_BASE+STM32_GPIO_OTYPER_OFFSET)
#  define STM32_GPIOK_OSPEED       (STM32_GPIOK_BASE+STM32_GPIO_OSPEED_OFFSET)
#  define STM32_GPIOK_PUPDR        (STM32_GPIOK_BASE+STM32_GPIO_PUPDR_OFFSET)
#  define STM32_GPIOK_IDR          (STM32_GPIOK_BASE+STM32_GPIO_IDR_OFFSET)
#  define STM32_GPIOK_ODR          (STM32_GPIOK_BASE+STM32_GPIO_ODR_OFFSET)
#  define STM32_GPIOK_BSRR         (STM32_GPIOK_BASE+STM32_GPIO_BSRR_OFFSET)
#  define STM32_GPIOK_LCKR         (STM32_GPIOK_BASE+STM32_GPIO_LCKR_OFFSET)
#  define STM32_GPIOK_AFRL         (STM32_GPIOK_BASE+STM32_GPIO_AFRL_OFFSET)
#  define STM32_GPIOK_AFRH         (STM32_GPIOK_BASE+STM32_GPIO_AFRH_OFFSET)
#endif

/* Register Bitfield Definitions ****************************************************/

/* GPIO port mode register */

#define GPIO_MODER_INPUT           (0) /* Input */
#define GPIO_MODER_OUTPUT          (1) /* General purpose output mode */
#define GPIO_MODER_ALT             (2) /* Alternate mode */
#define GPIO_MODER_ANALOG          (3) /* Analog mode */

#define GPIO_MODER_SHIFT(n)        ((n) << 1)
#define GPIO_MODER_MASK(n)         (3 << GPIO_MODER_SHIFT(n))

#define GPIO_MODER0_SHIFT          (0)
#define GPIO_MODER0_MASK           (3 << GPIO_MODER0_SHIFT)
#define GPIO_MODER1_SHIFT          (2)
#define GPIO_MODER1_MASK           (3 << GPIO_MODER1_SHIFT)
#define GPIO_MODER2_SHIFT          (4)
#define GPIO_MODER2_MASK           (3 << GPIO_MODER2_SHIFT)
#define GPIO_MODER3_SHIFT          (6)
#define GPIO_MODER3_MASK           (3 << GPIO_MODER3_SHIFT)
#define GPIO_MODER4_SHIFT          (8)
#define GPIO_MODER4_MASK           (3 << GPIO_MODER4_SHIFT)
#define GPIO_MODER5_SHIFT          (10)
#define GPIO_MODER5_MASK           (3 << GPIO_MODER5_SHIFT)
#define GPIO_MODER6_SHIFT          (12)
#define GPIO_MODER6_MASK           (3 << GPIO_MODER6_SHIFT)
#define GPIO_MODER7_SHIFT          (14)
#define GPIO_MODER7_MASK           (3 << GPIO_MODER7_SHIFT)
#define GPIO_MODER8_SHIFT          (16)
#define GPIO_MODER8_MASK           (3 << GPIO_MODER8_SHIFT)
#define GPIO_MODER9_SHIFT          (18)
#define GPIO_MODER9_MASK           (3 << GPIO_MODER9_SHIFT)
#define GPIO_MODER10_SHIFT         (20)
#define GPIO_MODER10_MASK          (3 << GPIO_MODER10_SHIFT)
#define GPIO_MODER11_SHIFT         (22)
#define GPIO_MODER11_MASK          (3 << GPIO_MODER11_SHIFT)
#define GPIO_MODER12_SHIFT         (24)
#define GPIO_MODER12_MASK          (3 << GPIO_MODER12_SHIFT)
#define GPIO_MODER13_SHIFT         (26)
#define GPIO_MODER13_MASK          (3 << GPIO_MODER13_SHIFT)
#define GPIO_MODER14_SHIFT         (28)
#define GPIO_MODER14_MASK          (3 << GPIO_MODER14_SHIFT)
#define GPIO_MODER15_SHIFT         (30)
#define GPIO_MODER15_MASK          (3 << GPIO_MODER15_SHIFT)

/* GPIO port output type register */

#define GPIO_OTYPER_OD(n)          (1 << (n)) /* 1=Output open-drain */
#define GPIO_OTYPER_PP(n)          (0)        /* 0=Ouput push-pull */

/* GPIO port output speed register */

#define GPIO_OSPEED_2MHz           (0) /* 2 MHz Low speed */
#define GPIO_OSPEED_25MHz          (1) /* 25 MHz Medium speed */
#define GPIO_OSPEED_50MHz          (2) /* 50 MHz Fast speed */
#define GPIO_OSPEED_100MHz         (3) /* 100 MHz High speed on 30 pF (80 MHz Output max speed on 15 pF) */

#define GPIO_OSPEED_SHIFT(n)       ((n) << 1)
#define GPIO_OSPEED_MASK(n)        (3 << GPIO_OSPEED_SHIFT(n))

#define GPIO_OSPEED0_SHIFT         (0)
#define GPIO_OSPEED0_MASK          (3 << GPIO_OSPEED0_SHIFT)
#define GPIO_OSPEED1_SHIFT         (2)
#define GPIO_OSPEED1_MASK          (3 << GPIO_OSPEED1_SHIFT)
#define GPIO_OSPEED2_SHIFT         (4)
#define GPIO_OSPEED2_MASK          (3 << GPIO_OSPEED2_SHIFT)
#define GPIO_OSPEED3_SHIFT         (6)
#define GPIO_OSPEED3_MASK          (3 << GPIO_OSPEED3_SHIFT)
#define GPIO_OSPEED4_SHIFT         (8)
#define GPIO_OSPEED4_MASK          (3 << GPIO_OSPEED4_SHIFT)
#define GPIO_OSPEED5_SHIFT         (10)
#define GPIO_OSPEED5_MASK          (3 << GPIO_OSPEED5_SHIFT)
#define GPIO_OSPEED6_SHIFT         (12)
#define GPIO_OSPEED6_MASK          (3 << GPIO_OSPEED6_SHIFT)
#define GPIO_OSPEED7_SHIFT         (14)
#define GPIO_OSPEED7_MASK          (3 << GPIO_OSPEED7_SHIFT)
#define GPIO_OSPEED8_SHIFT         (16)
#define GPIO_OSPEED8_MASK          (3 << GPIO_OSPEED8_SHIFT)
#define GPIO_OSPEED9_SHIFT         (18)
#define GPIO_OSPEED9_MASK          (3 << GPIO_OSPEED9_SHIFT)
#define GPIO_OSPEED10_SHIFT        (20)
#define GPIO_OSPEED10_MASK         (3 << GPIO_OSPEED10_SHIFT)
#define GPIO_OSPEED11_SHIFT        (22)
#define GPIO_OSPEED11_MASK         (3 << GPIO_OSPEED11_SHIFT)
#define GPIO_OSPEED12_SHIFT        (24)
#define GPIO_OSPEED12_MASK         (3 << GPIO_OSPEED12_SHIFT)
#define GPIO_OSPEED13_SHIFT        (26)
#define GPIO_OSPEED13_MASK         (3 << GPIO_OSPEED13_SHIFT)
#define GPIO_OSPEED14_SHIFT        (28)
#define GPIO_OSPEED14_MASK         (3 << GPIO_OSPEED14_SHIFT)
#define GPIO_OSPEED15_SHIFT        (30)
#define GPIO_OSPEED15_MASK         (3 << GPIO_OSPEED15_SHIFT)

/* GPIO port pull-up/pull-down register */

#define GPIO_PUPDR_NONE            (0) /* No pull-up, pull-down */
#define GPIO_PUPDR_PULLUP          (1) /* Pull-up */
#define GPIO_PUPDR_PULLDOWN        (2) /* Pull-down */

#define GPIO_PUPDR_SHIFT(n)        ((n) << 1)
#define GPIO_PUPDR_MASK(n)         (3 << GPIO_PUPDR_SHIFT(n))

#define GPIO_PUPDR0_SHIFT          (0)
#define GPIO_PUPDR0_MASK           (3 << GPIO_PUPDR0_SHIFT)
#define GPIO_PUPDR1_SHIFT          (2)
#define GPIO_PUPDR1_MASK           (3 << GPIO_PUPDR1_SHIFT)
#define GPIO_PUPDR2_SHIFT          (4)
#define GPIO_PUPDR2_MASK           (3 << GPIO_PUPDR2_SHIFT)
#define GPIO_PUPDR3_SHIFT          (6)
#define GPIO_PUPDR3_MASK           (3 << GPIO_PUPDR3_SHIFT)
#define GPIO_PUPDR4_SHIFT          (8)
#define GPIO_PUPDR4_MASK           (3 << GPIO_PUPDR4_SHIFT)
#define GPIO_PUPDR5_SHIFT          (10)
#define GPIO_PUPDR5_MASK           (3 << GPIO_PUPDR5_SHIFT)
#define GPIO_PUPDR6_SHIFT          (12)
#define GPIO_PUPDR6_MASK           (3 << GPIO_PUPDR6_SHIFT)
#define GPIO_PUPDR7_SHIFT          (14)
#define GPIO_PUPDR7_MASK           (3 << GPIO_PUPDR7_SHIFT)
#define GPIO_PUPDR8_SHIFT          (16)
#define GPIO_PUPDR8_MASK           (3 << GPIO_PUPDR8_SHIFT)
#define GPIO_PUPDR9_SHIFT          (18)
#define GPIO_PUPDR9_MASK           (3 << GPIO_PUPDR9_SHIFT)
#define GPIO_PUPDR10_SHIFT         (20)
#define GPIO_PUPDR10_MASK          (3 << GPIO_PUPDR10_SHIFT)
#define GPIO_PUPDR11_SHIFT         (22)
#define GPIO_PUPDR11_MASK          (3 << GPIO_PUPDR11_SHIFT)
#define GPIO_PUPDR12_SHIFT         (24)
#define GPIO_PUPDR12_MASK          (3 << GPIO_PUPDR12_SHIFT)
#define GPIO_PUPDR13_SHIFT         (26)
#define GPIO_PUPDR13_MASK          (3 << GPIO_PUPDR13_SHIFT)
#define GPIO_PUPDR14_SHIFT         (28)
#define GPIO_PUPDR14_MASK          (3 << GPIO_PUPDR14_SHIFT)
#define GPIO_PUPDR15_SHIFT         (30)
#define GPIO_PUPDR15_MASK          (3 << GPIO_PUPDR15_SHIFT)

/* GPIO port input data register */

#define GPIO_IDR(n)                (1 << (n))

/* GPIO port output data register */

#define GPIO_ODR(n)                (1 << (n))

/* GPIO port bit set/reset register */

#define GPIO_BSRR_SET(n)           (1 << (n))
#define GPIO_BSRR_RESET(n)         (1 << ((n)+16))

/* GPIO port configuration lock register */

#define GPIO_LCKR(n)               (1 << (n))
#define GPIO_LCKK                  (1 << 16)   /* Lock key */

/* GPIO alternate function low/high register */

#define GPIO_AFR_SHIFT(n)          ((n) << 2)
#define GPIO_AFR_MASK(n)           (15 << GPIO_AFR_SHIFT(n))

#define GPIO_AFRL0_SHIFT           (0)
#define GPIO_AFRL0_MASK            (15 << GPIO_AFRL0_SHIFT)
#define GPIO_AFRL1_SHIFT           (4)
#define GPIO_AFRL1_MASK            (15 << GPIO_AFRL1_SHIFT)
#define GPIO_AFRL2_SHIFT           (8)
#define GPIO_AFRL2_MASK            (15 << GPIO_AFRL2_SHIFT)
#define GPIO_AFRL3_SHIFT           (12)
#define GPIO_AFRL3_MASK            (15 << GPIO_AFRL3_SHIFT)
#define GPIO_AFRL4_SHIFT           (16)
#define GPIO_AFRL4_MASK            (15 << GPIO_AFRL4_SHIFT)
#define GPIO_AFRL5_SHIFT           (20)
#define GPIO_AFRL5_MASK            (15 << GPIO_AFRL5_SHIFT)
#define GPIO_AFRL6_SHIFT           (24)
#define GPIO_AFRL6_MASK            (15 << GPIO_AFRL6_SHIFT)
#define GPIO_AFRL7_SHIFT           (28)
#define GPIO_AFRL7_MASK            (15 << GPIO_AFRL7_SHIFT)

#define GPIO_AFRH8_SHIFT           (0)
#define GPIO_AFRH8_MASK            (15 << GPIO_AFRH8_SHIFT)
#define GPIO_AFRH9_SHIFT           (4)
#define GPIO_AFRH9_MASK            (15 << GPIO_AFRH9_SHIFT)
#define GPIO_AFRH10_SHIFT          (8)
#define GPIO_AFRH10_MASK           (15 << GPIO_AFRH10_SHIFT)
#define GPIO_AFRH11_SHIFT          (12)
#define GPIO_AFRH11_MASK           (15 << GPIO_AFRH11_SHIFT)
#define GPIO_AFRH12_SHIFT          (16)
#define GPIO_AFRH12_MASK           (15 << GPIO_AFRH12_SHIFT)
#define GPIO_AFRH13_SHIFT          (20)
#define GPIO_AFRH13_MASK           (15 << GPIO_AFRH13_SHIFT)
#define GPIO_AFRH14_SHIFT          (24)
#define GPIO_AFRH14_MASK           (15 << GPIO_AFRH14_SHIFT)
#define GPIO_AFRH15_SHIFT          (28)
#define GPIO_AFRH15_MASK           (15 << GPIO_AFRH15_SHIFT)


/* Bit-encoded input to stm32f4_configgpio() */

/* Each port bit of the general-purpose I/O (GPIO) ports can be individually configured
 * by software in several modes:
 *
 *  - Input floating
 *  - Input pull-up
 *  - Input-pull-down
 *  - Output open-drain with pull-up or pull-down capability
 *  - Output push-pull with pull-up or pull-down capability
 *  - Alternate function push-pull with pull-up or pull-down capability
 *  - Alternate function open-drain with pull-up or pull-down capability
 *  - Analog
 *
 * 20-bit Encoding:       1111 1111 1100 0000 0000
 *                        9876 5432 1098 7654 3210
 *                        ---- ---- ---- ---- ----
 * Inputs:                MMUU .... ...X PPPP BBBB
 * Outputs:               MMUU .... FFOV PPPP BBBB
 * Alternate Functions:   MMUU AAAA FFO. PPPP BBBB
 * Analog:                MM.. .... .... PPPP BBBB
 */

/* Mode:
 *
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * MM.. .... .... .... ....
 */

#define GPIO_MODE_SHIFT               (18)                       /* Bits 18-19: GPIO port mode */
#define GPIO_MODE_MASK                (3 << GPIO_MODE_SHIFT)
#  define GPIO_INPUT                  (0 << GPIO_MODE_SHIFT)     /* Input mode */
#  define GPIO_OUTPUT                 (1 << GPIO_MODE_SHIFT)     /* General purpose output mode */
#  define GPIO_ALT                    (2 << GPIO_MODE_SHIFT)     /* Alternate function mode */
#  define GPIO_ANALOG                 (3 << GPIO_MODE_SHIFT)     /* Analog mode */

/* Input/output pull-ups/downs (not used with analog):
 *
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * ..UU .... .... .... ....
 */

#define GPIO_PUPD_SHIFT               (16)                       /* Bits 16-17: Pull-up/pull down */
#define GPIO_PUPD_MASK                (3 << GPIO_PUPD_SHIFT)
#  define GPIO_FLOAT                  (0 << GPIO_PUPD_SHIFT)     /* No pull-up, pull-down */
#  define GPIO_PULLUP                 (1 << GPIO_PUPD_SHIFT)     /* Pull-up */
#  define GPIO_PULLDOWN               (2 << GPIO_PUPD_SHIFT)     /* Pull-down */

/* Alternate Functions:
 *
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * .... AAAA .... .... ....
 */

#define GPIO_AF_SHIFT                 (12)                       /* Bits 12-15: Alternate function */
#define GPIO_AF_MASK                  (15 << GPIO_AF_SHIFT)
#  define GPIO_AF(n)                  ((n) << GPIO_AF_SHIFT)
#  define GPIO_AF0                    (0 << GPIO_AF_SHIFT)
#  define GPIO_AF1                    (1 << GPIO_AF_SHIFT)
#  define GPIO_AF2                    (2 << GPIO_AF_SHIFT)
#  define GPIO_AF3                    (3 << GPIO_AF_SHIFT)
#  define GPIO_AF4                    (4 << GPIO_AF_SHIFT)
#  define GPIO_AF5                    (5 << GPIO_AF_SHIFT)
#  define GPIO_AF6                    (6 << GPIO_AF_SHIFT)
#  define GPIO_AF7                    (7 << GPIO_AF_SHIFT)
#  define GPIO_AF8                    (8 << GPIO_AF_SHIFT)
#  define GPIO_AF9                    (9 << GPIO_AF_SHIFT)
#  define GPIO_AF10                   (10 << GPIO_AF_SHIFT)
#  define GPIO_AF11                   (11 << GPIO_AF_SHIFT)
#  define GPIO_AF12                   (12 << GPIO_AF_SHIFT)
#  define GPIO_AF13                   (13 << GPIO_AF_SHIFT)
#  define GPIO_AF14                   (14 << GPIO_AF_SHIFT)
#  define GPIO_AF15                   (15 << GPIO_AF_SHIFT)

/* Output/Alt function frequency selection:
 *
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * .... .... FF.. .... ....
 */

#define GPIO_SPEED_SHIFT              (10)                       /* Bits 10-11: GPIO frequency selection */
#define GPIO_SPEED_MASK               (3 << GPIO_SPEED_SHIFT)
#  define GPIO_SPEED_2MHz             (0 << GPIO_SPEED_SHIFT)     /* 2 MHz Low speed output */
#  define GPIO_SPEED_25MHz            (1 << GPIO_SPEED_SHIFT)     /* 25 MHz Medium speed output */
#  define GPIO_SPEED_50MHz            (2 << GPIO_SPEED_SHIFT)     /* 50 MHz Fast speed output  */
#  define GPIO_SPEED_100MHz           (3 << GPIO_SPEED_SHIFT)     /* 100 MHz High speed output */

/* Output/Alt function type selection:
 *
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * .... .... ..O. .... ....
 */

#define GPIO_OPENDRAIN                (1 << 9)                   /* Bit9: 1=Open-drain output */
#define GPIO_PUSHPULL                 (0)                        /* Bit9: 0=Push-pull output */

/* If the pin is a GPIO digital output, then this identifies the initial output value.
 * If the pin is an input, this bit is overloaded to provide the qualifier to
 * distinquish input pull-up and -down:
 *
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * .... .... ...V .... ....
 */

#define GPIO_OUTPUT_SET               (1 << 8)                   /* Bit 8: If output, inital value of output */
#define GPIO_OUTPUT_CLEAR             (0)

/* External interrupt selection (GPIO inputs only):
 *
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * .... .... ...X .... ....
 */

#define GPIO_EXTI                     (1 << 8)                    /* Bit 8: Configure as EXTI interrupt */

/* This identifies the GPIO port:
 *
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * .... .... .... PPPP ....
 */

#define GPIO_PORT_SHIFT               (4)                        /* Bit 4-7:  Port number */
#define GPIO_PORT_MASK                (15 << GPIO_PORT_SHIFT)
#  define GPIO_PORTA                  (0 << GPIO_PORT_SHIFT)     /*   GPIOA */
#  define GPIO_PORTB                  (1 << GPIO_PORT_SHIFT)     /*   GPIOB */
#  define GPIO_PORTC                  (2 << GPIO_PORT_SHIFT)     /*   GPIOC */
#  define GPIO_PORTD                  (3 << GPIO_PORT_SHIFT)     /*   GPIOD */
#  define GPIO_PORTE                  (4 << GPIO_PORT_SHIFT)     /*   GPIOE */
#  define GPIO_PORTF                  (5 << GPIO_PORT_SHIFT)     /*   GPIOF */
#  define GPIO_PORTG                  (6 << GPIO_PORT_SHIFT)     /*   GPIOG */
#  define GPIO_PORTH                  (7 << GPIO_PORT_SHIFT)     /*   GPIOH */
#  define GPIO_PORTI                  (8 << GPIO_PORT_SHIFT)     /*   GPIOI */
#  define GPIO_PORTJ                  (9 << GPIO_PORT_SHIFT)     /*   GPIOJ */
#  define GPIO_PORTK                  (10 << GPIO_PORT_SHIFT)    /*   GPIOK */

/* This identifies the bit in the port:
 *
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * .... .... .... .... BBBB
 */

#define GPIO_PIN_SHIFT                (0)                        /* Bits 0-3: GPIO number: 0-15 */
#define GPIO_PIN_MASK                 (15 << GPIO_PIN_SHIFT)
#  define GPIO_PIN0                   (0 << GPIO_PIN_SHIFT)
#  define GPIO_PIN1                   (1 << GPIO_PIN_SHIFT)
#  define GPIO_PIN2                   (2 << GPIO_PIN_SHIFT)
#  define GPIO_PIN3                   (3 << GPIO_PIN_SHIFT)
#  define GPIO_PIN4                   (4 << GPIO_PIN_SHIFT)
#  define GPIO_PIN5                   (5 << GPIO_PIN_SHIFT)
#  define GPIO_PIN6                   (6 << GPIO_PIN_SHIFT)
#  define GPIO_PIN7                   (7 << GPIO_PIN_SHIFT)
#  define GPIO_PIN8                   (8 << GPIO_PIN_SHIFT)
#  define GPIO_PIN9                   (9 << GPIO_PIN_SHIFT)
#  define GPIO_PIN10                  (10 << GPIO_PIN_SHIFT)
#  define GPIO_PIN11                  (11 << GPIO_PIN_SHIFT)
#  define GPIO_PIN12                  (12 << GPIO_PIN_SHIFT)
#  define GPIO_PIN13                  (13 << GPIO_PIN_SHIFT)
#  define GPIO_PIN14                  (14 << GPIO_PIN_SHIFT)
#  define GPIO_PIN15                  (15 << GPIO_PIN_SHIFT)

/************************************************************************************
 * Public Data
 ************************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/* Base addresses for each GPIO block */

EXTERN const uint32_t g_gpiobase[STM32_NGPIO_PORTS];

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

/************************************************************************************
 * Name: stm32f4_configgpio
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin.
 *   Once it is configured as Alternative (GPIO_ALT|GPIO_CNF_AFPP|...)
 *   function, it must be unconfigured with stm32_unconfiggpio() with
 *   the same cfgset first before it can be set to non-alternative function.
 *
 * Returned Value:
 *   OK on success
 *   ERROR on invalid port, or when pin is locked as ALT function.
 *
 ************************************************************************************/

int stm32f4_configgpio(uint32_t cfgset);

/************************************************************************************
 * Name: stm32f4_unconfiggpio
 *
 * Description:
 *   Unconfigure a GPIO pin based on bit-encoded description of the pin, set it
 *   into default HiZ state (and possibly mark it's unused) and unlock it whether
 *   it was previsouly selected as alternative function (GPIO_ALT|GPIO_CNF_AFPP|...).
 *
 *   This is a safety function and prevents hardware from schocks, as unexpected
 *   write to the Timer Channel Output GPIO to fixed '1' or '0' while it should
 *   operate in PWM mode could produce excessive on-board currents and trigger
 *   over-current/alarm function.
 *
 * Returned Value:
 *  OK on success
 *  ERROR on invalid port
 *
 ************************************************************************************/

int stm32f4_unconfiggpio(uint32_t cfgset);

/************************************************************************************
 * Name: stm32f4_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ************************************************************************************/

void stm32f4_gpiowrite(uint32_t pinset, bool value);

/************************************************************************************
 * Name: stm32f4_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ************************************************************************************/

bool stm32f4_gpioread(uint32_t pinset);

/************************************************************************************
 * Name: stm32f4_gpiosetevent
 *
 * Description:
 *   Sets/clears GPIO based event and interrupt triggers.
 *
 * Input Parameters:
 *  - pinset: gpio pin configuration
 *  - rising/falling edge: enables
 *  - event:  generate event when set
 *  - func:   when non-NULL, generate interrupt
 *  - arg:    Argument passed to the interrupt callback
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure indicating the
 *   nature of the failure.
 *
 ************************************************************************************/

int stm32f4_gpiosetevent(uint32_t pinset, bool risingedge, bool fallingedge,
                       bool event, xcpt_t func, void *arg);

/************************************************************************************
 * Function:  stm32f4_dumpgpio
 *
 * Description:
 *   Dump all GPIO registers associated with the provided base address
 *
 ************************************************************************************/

#ifdef CONFIG_DEBUG_FEATURES
int stm32f4_dumpgpio(uint32_t pinset, const char *msg);
#else
#  define stm32f4_dumpgpio(p,m)
#endif

/************************************************************************************
 * Function:  stm32f4_gpioinit
 *
 * Description:
 *   Based on configuration within the .config file, it does:
 *    - Remaps positions of alternative functions.
 *
 *   Typically called from arm_boot()
 *
 ************************************************************************************/

void stm32f4_gpioinit(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_STM32F4_STM32F4_GPIO_H */