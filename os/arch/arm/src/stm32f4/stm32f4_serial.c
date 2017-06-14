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
/****************************************************************************
 * arch/arm/src/stm32f4/stm32f4_serial.c
 *
 *   Copyright (C) 2009-2010, 2012-2014 Gregory Nutt. All rights reserved.
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
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <tinyara/irq.h>
#include <tinyara/serial/serial.h>

#ifdef CONFIG_SERIAL_TERMIOS
#include <termios.h>
#endif

#include <arch/serial.h>
#include <arch/board/board.h>

#include "chip.h"

#include "up_arch.h"
#include "up_internal.h"

#include "stm32f4.h"
#include "stm32f4_rcc.h"
#include "stm32f4_gpio.h"
#include "stm32f4_uart.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#undef TTYS1_DEV
#undef TTYS2_DEV
#undef TTYS3_DEV
#undef TTYS4_DEV
#undef TTYS5_DEV
#undef TTYS6_DEV
#undef TTYS7_DEV
#undef TTYS8_DEV

/* Which UART with be ttyS0/console and which ttyS1? ttyS2? ... ttyS7 */

#define USART1_DEV      g_usart1priv.dev
#define USART2_DEV      g_usart2priv.dev
#define USART3_DEV      g_usart3priv.dev
#define UART4_DEV       g_uart4priv.dev
#define UART5_DEV       g_uart5priv.dev
#define USART6_DEV      g_usart6priv.dev
#define UART7_DEV       g_uart7priv.dev
#define UART8_DEV       g_uart8priv.dev

/* Select USART parameters for the selected console */

#ifdef HAVE_SERIAL_CONSOLE
#if defined(CONFIG_USART1_SERIAL_CONSOLE)
#define STM32_CONSOLE_BASE      STM32_USART1_BASE
#define STM32_APBCLOCK          STM32_PCLK2_FREQUENCY
#define STM32_CONSOLE_APBREG    STM32_RCC_APB2ENR
#define STM32_CONSOLE_APBEN     RCC_APB2ENR_USART1EN
#define STM32_CONSOLE_BAUD      CONFIG_USART1_BAUD
#define STM32_CONSOLE_BITS      CONFIG_USART1_BITS
#define STM32_CONSOLE_PARITY    CONFIG_USART1_PARITY
#define STM32_CONSOLE_2STOP     CONFIG_USART1_2STOP
#define STM32_CONSOLE_TX        GPIO_USART1_TX
#define STM32_CONSOLE_RX        GPIO_USART1_RX
#define CONSOLE_DEV             USART1_DEV  
#elif defined(CONFIG_USART2_SERIAL_CONSOLE)
#define STM32_CONSOLE_BASE      STM32_USART2_BASE
#define STM32_APBCLOCK          STM32_PCLK1_FREQUENCY
#define STM32_CONSOLE_APBREG    STM32_RCC_APB1ENR
#define STM32_CONSOLE_APBEN     RCC_APB1ENR_USART2EN
#define STM32_CONSOLE_BAUD      CONFIG_USART2_BAUD
#define STM32_CONSOLE_BITS      CONFIG_USART2_BITS
#define STM32_CONSOLE_PARITY    CONFIG_USART2_PARITY
#define STM32_CONSOLE_2STOP     CONFIG_USART2_2STOP
#define STM32_CONSOLE_TX        GPIO_USART2_TX
#define STM32_CONSOLE_RX        GPIO_USART2_RX
#define CONSOLE_DEV             USART2_DEV  
#elif defined(CONFIG_USART3_SERIAL_CONSOLE)
#define STM32_CONSOLE_BASE      STM32_USART3_BASE
#define STM32_APBCLOCK          STM32_PCLK1_FREQUENCY
#define STM32_CONSOLE_APBREG    STM32_RCC_APB1ENR
#define STM32_CONSOLE_APBEN     RCC_APB1ENR_USART3EN
#define STM32_CONSOLE_BAUD      CONFIG_USART3_BAUD
#define STM32_CONSOLE_BITS      CONFIG_USART3_BITS
#define STM32_CONSOLE_PARITY    CONFIG_USART3_PARITY
#define STM32_CONSOLE_2STOP     CONFIG_USART3_2STOP
#define STM32_CONSOLE_TX        GPIO_USART3_TX
#define STM32_CONSOLE_RX        GPIO_USART3_RX
#define CONSOLE_DEV             USART3_DEV  
#elif defined(CONFIG_UART4_SERIAL_CONSOLE)
#define STM32_CONSOLE_BASE      STM32_UART4_BASE
#define STM32_APBCLOCK          STM32_PCLK1_FREQUENCY
#define STM32_CONSOLE_APBREG    STM32_RCC_APB1ENR
#define STM32_CONSOLE_APBEN     RCC_APB1ENR_UART4EN
#define STM32_CONSOLE_BAUD      CONFIG_UART4_BAUD
#define STM32_CONSOLE_BITS      CONFIG_UART4_BITS
#define STM32_CONSOLE_PARITY    CONFIG_UART4_PARITY
#define STM32_CONSOLE_2STOP     CONFIG_UART4_2STOP
#define STM32_CONSOLE_TX        GPIO_UART4_TX
#define STM32_CONSOLE_RX        GPIO_UART4_RX
#define CONSOLE_DEV             USART4_DEV  
#elif defined(CONFIG_UART5_SERIAL_CONSOLE)
#define STM32_CONSOLE_BASE      STM32_UART5_BASE
#define STM32_APBCLOCK          STM32_PCLK1_FREQUENCY
#define STM32_CONSOLE_APBREG    STM32_RCC_APB1ENR
#define STM32_CONSOLE_APBEN     RCC_APB1ENR_UART5EN
#define STM32_CONSOLE_BAUD      CONFIG_UART5_BAUD
#define STM32_CONSOLE_BITS      CONFIG_UART5_BITS
#define STM32_CONSOLE_PARITY    CONFIG_UART5_PARITY
#define STM32_CONSOLE_2STOP     CONFIG_UART5_2STOP
#define STM32_CONSOLE_TX        GPIO_UART5_TX
#define STM32_CONSOLE_RX        GPIO_UART5_RX
#define CONSOLE_DEV             USART5_DEV  
#elif defined(CONFIG_USART6_SERIAL_CONSOLE)
#define STM32_CONSOLE_BASE      STM32_USART6_BASE
#define STM32_APBCLOCK          STM32_PCLK2_FREQUENCY
#define STM32_CONSOLE_APBREG    STM32_RCC_APB2ENR
#define STM32_CONSOLE_APBEN     RCC_APB2ENR_USART6EN
#define STM32_CONSOLE_BAUD      CONFIG_USART6_BAUD
#define STM32_CONSOLE_BITS      CONFIG_USART6_BITS
#define STM32_CONSOLE_PARITY    CONFIG_USART6_PARITY
#define STM32_CONSOLE_2STOP     CONFIG_USART6_2STOP
#define STM32_CONSOLE_TX        GPIO_USART6_TX
#define STM32_CONSOLE_RX        GPIO_USART6_RX
#define CONSOLE_DEV             USART6_DEV  
#elif defined(CONFIG_UART7_SERIAL_CONSOLE)
#define STM32_CONSOLE_BASE      STM32_UART7_BASE
#define STM32_APBCLOCK          STM32_PCLK1_FREQUENCY
#define STM32_CONSOLE_APBREG    STM32_RCC_APB1ENR
#define STM32_CONSOLE_APBEN     RCC_APB1ENR_UART7EN
#define STM32_CONSOLE_BAUD      CONFIG_UART7_BAUD
#define STM32_CONSOLE_BITS      CONFIG_UART7_BITS
#define STM32_CONSOLE_PARITY    CONFIG_UART7_PARITY
#define STM32_CONSOLE_2STOP     CONFIG_UART7_2STOP
#define STM32_CONSOLE_TX        GPIO_UART7_TX
#define STM32_CONSOLE_RX        GPIO_UART7_RX
#define CONSOLE_DEV             USART7_DEV  
#elif defined(CONFIG_UART8_SERIAL_CONSOLE)
#define STM32_CONSOLE_BASE      STM32_UART8_BASE
#define STM32_APBCLOCK          STM32_PCLK1_FREQUENCY
#define STM32_CONSOLE_APBREG    STM32_RCC_APB1ENR
#define STM32_CONSOLE_APBEN     RCC_APB1ENR_UART8EN
#define STM32_CONSOLE_BAUD      CONFIG_UART8_BAUD
#define STM32_CONSOLE_BITS      CONFIG_UART8_BITS
#define STM32_CONSOLE_PARITY    CONFIG_UART8_PARITY
#define STM32_CONSOLE_2STOP     CONFIG_UART8_2STOP
#define STM32_CONSOLE_TX        GPIO_UART8_TX
#define STM32_CONSOLE_RX        GPIO_UART8_RX
#define CONSOLE_DEV             USART8_DEV  
#endif

/* CR1 settings */

#if STM32_CONSOLE_BITS == 9
#define USART_CR1_M_VALUE USART_CR1_M
#else
#define USART_CR1_M_VALUE 0
#endif

#if STM32_CONSOLE_PARITY == 1
#define USART_CR1_PARITY_VALUE (USART_CR1_PCE|USART_CR1_PS)
#elif STM32_CONSOLE_PARITY == 2
#define USART_CR1_PARITY_VALUE USART_CR1_PCE
#else
#define USART_CR1_PARITY_VALUE 0
#endif

#define USART_CR1_CLRBITS\
    (USART_CR1_M | USART_CR1_PCE | USART_CR1_PS | USART_CR1_TE | \
     USART_CR1_RE | USART_CR1_ALLINTS)

#define USART_CR1_SETBITS (USART_CR1_M_VALUE|USART_CR1_PARITY_VALUE)

/* CR2 settings */

#if STM32_CONSOLE_2STOP != 0
#define USART_CR2_STOP2_VALUE USART_CR2_STOP2
#else
#define USART_CR2_STOP2_VALUE 0
#endif

#define USART_CR2_CLRBITS \
    (USART_CR2_STOP_MASK | USART_CR2_CLKEN | USART_CR2_CPOL | USART_CR2_CPHA | \
     USART_CR2_LBCL | USART_CR2_LBDIE)
#define USART_CR2_SETBITS USART_CR2_STOP2_VALUE

/* CR3 settings */

#define USART_CR3_CLRBITS \
    (USART_CR3_CTSIE | USART_CR3_CTSE | USART_CR3_RTSE | USART_CR3_EIE)
#define USART_CR3_SETBITS 0



