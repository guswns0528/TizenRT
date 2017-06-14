/************************************************************************************
 * arch/arm/src/stm32f4/stm32f4_uart.h
 *
 *   Copyright (C) 2009, 2012-2013, 2015 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32F4_STM32F4_UART_H
#define __ARCH_ARM_SRC_STM32F4_STM32F4_UART_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <tinyara/config.h>

#include "chip.h"
#include "chip/stm32f427_uart.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Make sure that we have not enabled more U[S]ARTs than are supported by the
 * device.
 */

#if STM32_NUSART < 8 || !defined(CONFIG_STM32_HAVE_UART8)
#  undef CONFIG_STM32_UART8
#endif
#if STM32_NUSART < 7 || !defined(CONFIG_STM32_HAVE_UART7)
#  undef CONFIG_STM32_UART7
#endif
#if STM32_NUSART < 6 || !defined(CONFIG_STM32_HAVE_USART6)
#  undef CONFIG_STM32_USART6
#endif
#if STM32_NUSART < 5 || !defined(CONFIG_STM32_HAVE_UART5)
#  undef CONFIG_STM32_UART5
#endif
#if STM32_NUSART < 4 || !defined(CONFIG_STM32_HAVE_UART4)
#  undef CONFIG_STM32_UART4
#endif
#if STM32_NUSART < 3 || !defined(CONFIG_STM32_HAVE_USART3)
#  undef CONFIG_STM32_USART3
#endif
#if STM32_NUSART < 2
#  undef CONFIG_STM32_USART2
#endif
#if STM32_NUSART < 1
#  undef CONFIG_STM32_USART1
#endif

/* Is there a serial console? */

#if defined(CONFIG_USART1_SERIAL_CONSOLE)
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_USART3_SERIAL_CONSOLE
#  undef CONFIG_UART4_SERIAL_CONSOLE
#  undef CONFIG_UART5_SERIAL_CONSOLE
#  undef CONFIG_USART6_SERIAL_CONSOLE
#  undef CONFIG_UART7_SERIAL_CONSOLE
#  undef CONFIG_UART8_SERIAL_CONSOLE
#  define CONSOLE_UART 1
#  define HAVE_SERIAL_CONSOLE 1
#elif defined(CONFIG_USART2_SERIAL_CONSOLE)
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART3_SERIAL_CONSOLE
#  undef CONFIG_UART4_SERIAL_CONSOLE
#  undef CONFIG_UART5_SERIAL_CONSOLE
#  undef CONFIG_USART6_SERIAL_CONSOLE
#  undef CONFIG_UART7_SERIAL_CONSOLE
#  undef CONFIG_UART8_SERIAL_CONSOLE
#  define CONSOLE_UART 2
#  define HAVE_SERIAL_CONSOLE 1
#elif defined(CONFIG_USART3_SERIAL_CONSOLE)
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_UART4_SERIAL_CONSOLE
#  undef CONFIG_UART5_SERIAL_CONSOLE
#  undef CONFIG_USART6_SERIAL_CONSOLE
#  undef CONFIG_UART7_SERIAL_CONSOLE
#  undef CONFIG_UART8_SERIAL_CONSOLE
#  define CONSOLE_UART 3
#  define HAVE_SERIAL_CONSOLE 1
#elif defined(CONFIG_UART4_SERIAL_CONSOLE)
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_USART3_SERIAL_CONSOLE
#  undef CONFIG_UART5_SERIAL_CONSOLE
#  undef CONFIG_USART6_SERIAL_CONSOLE
#  undef CONFIG_UART7_SERIAL_CONSOLE
#  undef CONFIG_UART8_SERIAL_CONSOLE
#  define CONSOLE_UART 4
#  define HAVE_SERIAL_CONSOLE 1
#elif defined(CONFIG_UART5_SERIAL_CONSOLE)
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_USART3_SERIAL_CONSOLE
#  undef CONFIG_UART4_SERIAL_CONSOLE
#  undef CONFIG_USART6_SERIAL_CONSOLE
#  undef CONFIG_UART7_SERIAL_CONSOLE
#  undef CONFIG_UART8_SERIAL_CONSOLE
#  define CONSOLE_UART 5
#  define HAVE_SERIAL_CONSOLE 1
#elif defined(CONFIG_USART6_SERIAL_CONSOLE)
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_USART3_SERIAL_CONSOLE
#  undef CONFIG_UART4_SERIAL_CONSOLE
#  undef CONFIG_UART5_SERIAL_CONSOLE
#  undef CONFIG_UART7_SERIAL_CONSOLE
#  undef CONFIG_UART8_SERIAL_CONSOLE
#  define CONSOLE_UART 6
#  define HAVE_SERIAL_CONSOLE 1
#elif defined(CONFIG_UART7_SERIAL_CONSOLE)
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_USART3_SERIAL_CONSOLE
#  undef CONFIG_UART4_SERIAL_CONSOLE
#  undef CONFIG_UART5_SERIAL_CONSOLE
#  undef CONFIG_USART6_SERIAL_CONSOLE
#  undef CONFIG_UART5_SERIAL_CONSOLE
#  undef CONFIG_UART8_SERIAL_CONSOLE
#  define CONSOLE_UART 7
#  define HAVE_SERIAL_CONSOLE 1
#elif defined(CONFIG_UART8_SERIAL_CONSOLE)
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_USART3_SERIAL_CONSOLE
#  undef CONFIG_UART4_SERIAL_CONSOLE
#  undef CONFIG_UART6_SERIAL_CONSOLE
#  undef CONFIG_USART6_SERIAL_CONSOLE
#  undef CONFIG_UART7_SERIAL_CONSOLE
#  define CONSOLE_UART 8
#  define HAVE_SERIAL_CONSOLE 1
#else
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  undef CONFIG_USART2_SERIAL_CONSOLE
#  undef CONFIG_USART3_SERIAL_CONSOLE
#  undef CONFIG_UART4_SERIAL_CONSOLE
#  undef CONFIG_UART5_SERIAL_CONSOLE
#  undef CONFIG_USART6_SERIAL_CONSOLE
#  undef CONFIG_UART7_SERIAL_CONSOLE
#  undef CONFIG_UART8_SERIAL_CONSOLE
#  define CONSOLE_UART 0
#  undef HAVE_SERIAL_CONSOLE
#endif

#define USART_CR1_USED_INTS    (USART_CR1_RXNEIE | USART_CR1_TXEIE | USART_CR1_PEIE)

/************************************************************************************
 * Public Types
 ************************************************************************************/

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

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_STM32F4_STM32F4_UART_H */
