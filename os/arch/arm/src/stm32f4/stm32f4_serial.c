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


/* Calculate USART BAUD rate divider */

/* The baud rate for the receiver and transmitter (Rx and Tx) are both set
 * to the same value as programmed in the Mantissa and Fraction values of
 * USARTDIV.
 *
 *   baud     = fCK / (16 * usartdiv)
 *   usartdiv = fCK / (16 * baud)
 *
 * Where fCK is the input clock to the peripheral (PCLK1 for USART2, 3, 4,
 * 5 or PCLK2 for USART1).  Example, fCK=72MHz baud=115200,
 * usartdiv=39.0625=39 1/16th;
 *
 * First calculate:
 *
 *   usartdiv32 = 32 * usartdiv = fCK / (baud/2)
 *
 * (NOTE: all standard baud values are even so dividing by two does not
 * lose precision).  Eg. (same fCK and buad), usartdiv32 = 1250
 */

#define STM32_USARTDIV32 (STM32_APBCLOCK / (STM32_CONSOLE_BAUD >> 1))

/* The mantissa is then usartdiv32 / 32:
 *
 *   mantissa = usartdiv32 / 32/
 *
 * Eg. usartdiv32=1250, mantissa = 39
 */

#define STM32_MANTISSA (STM32_USARTDIV32 >> 5)

/* And the fraction:
 *
 *  fraction = (usartdiv32 - mantissa*32 + 1) / 2
 *
 * Eg., (1,250 - 39*32 + 1)/2 = 1 (or 0.0625)
 */

#define STM32_FRACTION \
    ((STM32_USARTDIV32 - (STM32_MANTISSA << 5) + 1) >> 1)

/* And, finally, the BRR value is: */

#define STM32_BRR_VALUE \
    ((STM32_MANTISSA << USART_BRR_MANT_SHIFT) | \
     (STM32_FRACTION << USART_BRR_FRAC_SHIFT))

#endif /* HAVE_SERIAL_CONSOLE */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct up_dev_s
{
    struct uart_dev_s dev;       /* Generic UART device */
    uint16_t          ie;        /* Saved interrupt mask bits value */
    uint16_t          sr;        /* Saved status bits */

    /* If termios are supported, then the following fields may vary at
     * runtime.
     */

#ifdef CONFIG_SERIAL_TERMIOS
    uint8_t           parity;    /* 0=none, 1=odd, 2=even */
    uint8_t           bits;      /* Number of bits (7 or 8) */
    bool              stopbits2; /* True: Configure with 2 stop bits instead of 1 */
#ifdef CONFIG_SERIAL_IFLOWCONTROL
    bool              iflow;     /* input flow control (RTS) enabled */
#endif
#ifdef CONFIG_SERIAL_OFLOWCONTROL
    bool              oflow;     /* output flow control (CTS) enabled */
#endif
    uint32_t          baud;      /* Configured baud */
#else
    const uint8_t     parity;    /* 0=none, 1=odd, 2=even */
    const uint8_t     bits;      /* Number of bits (7 or 8) */
    const bool        stopbits2; /* True: Configure with 2 stop bits instead of 1 */
#ifdef CONFIG_SERIAL_IFLOWCONTROL
    const bool        iflow;     /* input flow control (RTS) enabled */
#endif
#ifdef CONFIG_SERIAL_OFLOWCONTROL
    const bool        oflow;     /* output flow control (CTS) enabled */
#endif
    const uint32_t    baud;      /* Configured baud */
#endif

    const uint8_t     irq;       /* IRQ associated with this USART */
    const uint32_t    apbclock;  /* PCLK 1 or 2 frequency */
    const uint32_t    usartbase; /* Base address of USART registers */
    const uint32_t    tx_gpio;   /* U[S]ART TX GPIO pin configuration */
    const uint32_t    rx_gpio;   /* U[S]ART RX GPIO pin configuration */
#ifdef CONFIG_SERIAL_IFLOWCONTROL
    const uint32_t    rts_gpio;  /* U[S]ART RTS GPIO pin configuration */
#endif
#ifdef CONFIG_SERIAL_OFLOWCONTROL
    const uint32_t    cts_gpio;  /* U[S]ART CTS GPIO pin configuration */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void up_set_format(struct uart_dev_s *dev);
static int  up_setup(struct uart_dev_s *dev);
static void up_shutdown(struct uart_dev_s *dev);
static int  up_attach(struct uart_dev_s *dev);
static void up_detach(struct uart_dev_s *dev);
static int  up_interrupt(int irq, void *context, void *arg);
static int  up_ioctl(struct file *filep, int cmd, unsigned long arg);
static int  up_receive(struct uart_dev_s *dev, unsigned int *status);
static void up_rxint(struct uart_dev_s *dev, bool enable);
static bool up_rxavailable(struct uart_dev_s *dev);
#ifdef CONFIG_SERIAL_IFLOWCONTROL
static bool up_rxflowcontrol(struct uart_dev_s *dev, unsigned int nbuffered,
        bool upper);
#endif
static void up_send(struct uart_dev_s *dev, int ch);
static void up_txint(struct uart_dev_s *dev, bool enable);
static bool up_txready(struct uart_dev_s *dev);

#ifdef CONFIG_PM
static void up_pm_notify(struct pm_callback_s *cb, int dowmin,
        enum pm_state_e pmstate);
static int  up_pm_prepare(struct pm_callback_s *cb, int domain,
        enum pm_state_e pmstate);
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct uart_ops_s g_uart_ops =
{
    .setup          = up_setup,
    .shutdown       = up_shutdown,
    .attach         = up_attach,
    .detach         = up_detach,
    .ioctl          = up_ioctl,
    .receive        = up_receive,
    .rxint          = up_rxint,
    .rxavailable    = up_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
    .rxflowcontrol  = up_rxflowcontrol,
#endif
    .send           = up_send,
    .txint          = up_txint,
    .txready        = up_txready,
    .txempty        = up_txready,
};

/* I/O buffers */

#if defined(CONFIG_STM32_USART1)
static char g_usart1rxbuffer[CONFIG_USART1_RXBUFSIZE];
static char g_usart1txbuffer[CONFIG_USART1_TXBUFSIZE];
#endif

#if defined(CONFIG_STM32_USART2)
static char g_usart2rxbuffer[CONFIG_USART2_RXBUFSIZE];
static char g_usart2txbuffer[CONFIG_USART2_TXBUFSIZE];
#endif

#if defined(CONFIG_STM32_USART3)
static char g_usart3rxbuffer[CONFIG_USART3_RXBUFSIZE];
static char g_usart3txbuffer[CONFIG_USART3_TXBUFSIZE];
#endif

#if defined(CONFIG_STM32_UART4)
static char g_uart4rxbuffer[CONFIG_UART4_RXBUFSIZE];
static char g_uart4txbuffer[CONFIG_UART4_TXBUFSIZE];
#endif

#if defined(CONFIG_STM32_UART5)
static char g_uart5rxbuffer[CONFIG_UART5_RXBUFSIZE];
static char g_uart5txbuffer[CONFIG_UART5_TXBUFSIZE];
#endif

#if defined(CONFIG_STM32_USART6)
static char g_usart6rxbuffer[CONFIG_USART6_RXBUFSIZE];
static char g_usart6txbuffer[CONFIG_USART6_TXBUFSIZE];
#endif

#if defined(CONFIG_STM32_UART7)
static char g_uart7rxbuffer[CONFIG_UART7_RXBUFSIZE];
static char g_uart7txbuffer[CONFIG_UART7_TXBUFSIZE];
#endif

#if defined(CONFIG_STM32_UART8)
static char g_uart8rxbuffer[CONFIG_UART8_RXBUFSIZE];
static char g_uart8txbuffer[CONFIG_UART8_TXBUFSIZE];
#endif

/* This describes the state of the STM32 USART1 ports. */

#if defined(CONFIG_STM32_USART1)
static struct up_dev_s g_usart1priv =
{
    .dev =
    {
#if CONSOLE_UART == 1
        .isconsole = true,
#endif
        .recv      =
        {
            .size    = CONFIG_USART1_RXBUFSIZE,
            .buffer  = g_usart1rxbuffer,
        },
        .xmit      =
        {
            .size    = CONFIG_USART1_TXBUFSIZE,
            .buffer  = g_usart1txbuffer,
        },
        .ops       = &g_uart_ops,
        .priv      = &g_usart1priv,
    },

    .irq           = STM32_IRQ_USART1,
    .parity        = CONFIG_USART1_PARITY,
    .bits          = CONFIG_USART1_BITS,
    .stopbits2     = CONFIG_USART1_2STOP,
    .baud          = CONFIG_USART1_BAUD,
    .apbclock      = STM32_PCLK2_FREQUENCY,
    .usartbase     = STM32_USART1_BASE,
    .tx_gpio       = GPIO_USART1_TX,
    .rx_gpio       = GPIO_USART1_RX,
#if defined(CONFIG_SERIAL_OFLOWCONTROL) && defined(CONFIG_USART1_OFLOWCONTROL)
    .oflow         = true,
    .cts_gpio      = GPIO_USART1_CTS,
#endif
#if defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_USART1_IFLOWCONTROL)
    .iflow         = true,
    .rts_gpio      = GPIO_USART1_RTS,
#endif
};
#endif

/* This describes the state of the STM32 USART2 port. */

#if defined(CONFIG_STM32_USART2)
static struct up_dev_s g_usart2priv =
{
    .dev =
    {
#if CONSOLE_UART == 2
        .isconsole = true,
#endif
        .recv      =
        {
            .size    = CONFIG_USART2_RXBUFSIZE,
            .buffer  = g_usart2rxbuffer,
        },
        .xmit      =
        {
            .size    = CONFIG_USART2_TXBUFSIZE,
            .buffer  = g_usart2txbuffer,
        },
        .ops       = &g_uart_ops,
        .priv      = &g_usart2priv,
    },

    .irq           = STM32_IRQ_USART2,
    .parity        = CONFIG_USART2_PARITY,
    .bits          = CONFIG_USART2_BITS,
    .stopbits2     = CONFIG_USART2_2STOP,
    .baud          = CONFIG_USART2_BAUD,
    .apbclock      = STM32_PCLK1_FREQUENCY,
    .usartbase     = STM32_USART2_BASE,
    .tx_gpio       = GPIO_USART2_TX,
    .rx_gpio       = GPIO_USART2_RX,
#if defined(CONFIG_SERIAL_OFLOWCONTROL) && defined(CONFIG_USART2_OFLOWCONTROL)
    .oflow         = true,
    .cts_gpio      = GPIO_USART2_CTS,
#endif
#if defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_USART2_IFLOWCONTROL)
    .iflow         = true,
    .rts_gpio      = GPIO_USART2_RTS,
#endif
};
#endif

/* This describes the state of the STM32 USART3 port. */

#if defined(CONFIG_STM32_USART3)
static struct up_dev_s g_usart3priv =
{
    .dev =
    {
#if CONSOLE_UART == 3
        .isconsole = true,
#endif
        .recv      =
        {
            .size    = CONFIG_USART3_RXBUFSIZE,
            .buffer  = g_usart3rxbuffer,
        },
        .xmit      =
        {
            .size    = CONFIG_USART3_TXBUFSIZE,
            .buffer  = g_usart3txbuffer,
        },
        .ops       = &g_uart_ops,
        .priv      = &g_usart3priv,
    },

    .irq           = STM32_IRQ_USART3,
    .parity        = CONFIG_USART3_PARITY,
    .bits          = CONFIG_USART3_BITS,
    .stopbits2     = CONFIG_USART3_2STOP,
    .baud          = CONFIG_USART3_BAUD,
    .apbclock      = STM32_PCLK1_FREQUENCY,
    .usartbase     = STM32_USART3_BASE,
    .tx_gpio       = GPIO_USART3_TX,
    .rx_gpio       = GPIO_USART3_RX,
#if defined(CONFIG_SERIAL_OFLOWCONTROL) && defined(CONFIG_USART3_OFLOWCONTROL)
    .oflow         = true,
    .cts_gpio      = GPIO_USART3_CTS,
#endif
#if defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_USART3_IFLOWCONTROL)
    .iflow         = true,
    .rts_gpio      = GPIO_USART3_RTS,
#endif
};
#endif

/* This describes the state of the STM32 UART4 port. */

#if defined(CONFIG_STM32_UART4)
static struct up_dev_s g_uart4priv =
{
    .dev =
    {
#if CONSOLE_UART == 4
        .isconsole = true,
#endif
        .recv      =
        {
            .size    = CONFIG_UART4_RXBUFSIZE,
            .buffer  = g_uart4rxbuffer,
        },
        .xmit      =
        {
            .size    = CONFIG_UART4_TXBUFSIZE,
            .buffer  = g_uart4txbuffer,
        },
        .ops       = &g_uart_ops,
        .priv      = &g_uart4priv,
    },

    .irq           = STM32_IRQ_UART4,
    .parity        = CONFIG_UART4_PARITY,
    .bits          = CONFIG_UART4_BITS,
    .stopbits2     = CONFIG_UART4_2STOP,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
    .iflow         = false,
#endif
#ifdef CONFIG_SERIAL_OFLOWCONTROL
    .oflow         = false,
#endif
    .baud          = CONFIG_UART4_BAUD,
    .apbclock      = STM32_PCLK1_FREQUENCY,
    .usartbase     = STM32_UART4_BASE,
    .tx_gpio       = GPIO_UART4_TX,
    .rx_gpio       = GPIO_UART4_RX,
#ifdef CONFIG_SERIAL_OFLOWCONTROL
    .cts_gpio      = 0,
#endif
#ifdef CONFIG_SERIAL_IFLOWCONTROL
    .rts_gpio      = 0,
#endif
};
#endif

/* This describes the state of the STM32 UART5 port. */

#if defined(CONFIG_STM32_UART5)
static struct up_dev_s g_uart5priv =
{
    .dev =
    {
#if CONSOLE_UART == 5
        .isconsole = true,
#endif
        .recv     =
        {
            .size   = CONFIG_UART5_RXBUFSIZE,
            .buffer = g_uart5rxbuffer,
        },
        .xmit     =
        {
            .size   = CONFIG_UART5_TXBUFSIZE,
            .buffer = g_uart5txbuffer,
        },
        .ops      = &g_uart_ops,
        .priv     = &g_uart5priv,
    },

    .irq            = STM32_IRQ_UART5,
    .parity         = CONFIG_UART5_PARITY,
    .bits           = CONFIG_UART5_BITS,
    .stopbits2      = CONFIG_UART5_2STOP,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
    .iflow         = false,
#endif
#ifdef CONFIG_SERIAL_OFLOWCONTROL
    .oflow         = false,
#endif
    .baud           = CONFIG_UART5_BAUD,
    .apbclock       = STM32_PCLK1_FREQUENCY,
    .usartbase      = STM32_UART5_BASE,
    .tx_gpio        = GPIO_UART5_TX,
    .rx_gpio        = GPIO_UART5_RX,
#ifdef CONFIG_SERIAL_OFLOWCONTROL
    .cts_gpio      = 0,
#endif
#ifdef CONFIG_SERIAL_IFLOWCONTROL
    .rts_gpio      = 0,
#endif
};
#endif

/* This describes the state of the STM32 USART6 port. */

#if defined(CONFIG_STM32_USART6)
static struct up_dev_s g_usart6priv =
{
    .dev =
    {
#if CONSOLE_UART == 6
        .isconsole = true,
#endif
        .recv     =
        {
            .size   = CONFIG_USART6_RXBUFSIZE,
            .buffer = g_usart6rxbuffer,
        },
        .xmit     =
        {
            .size   = CONFIG_USART6_TXBUFSIZE,
            .buffer = g_usart6txbuffer,
        },
        .ops      = &g_uart_ops,
        .priv     = &g_usart6priv,
    },

    .irq            = STM32_IRQ_USART6,
    .parity         = CONFIG_USART6_PARITY,
    .bits           = CONFIG_USART6_BITS,
    .stopbits2      = CONFIG_USART6_2STOP,
    .baud           = CONFIG_USART6_BAUD,
    .apbclock       = STM32_PCLK2_FREQUENCY,
    .usartbase      = STM32_USART6_BASE,
    .tx_gpio        = GPIO_USART6_TX,
    .rx_gpio        = GPIO_USART6_RX,
#if defined(CONFIG_SERIAL_OFLOWCONTROL) && defined(CONFIG_USART6_OFLOWCONTROL)
    .oflow          = true,
    .cts_gpio       = GPIO_USART6_CTS,
#endif
#if defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_USART6_IFLOWCONTROL)
    .iflow          = true,
    .rts_gpio       = GPIO_USART6_RTS,
#endif
};
#endif

/* This describes the state of the STM32 UART7 port. */

#if defined(CONFIG_STM32_UART7)
static struct up_dev_s g_uart7priv =
{
    .dev =
    {
#if CONSOLE_UART == 7
        .isconsole = true,
#endif
        .recv     =
        {
            .size   = CONFIG_UART7_RXBUFSIZE,
            .buffer = g_uart7rxbuffer,
        },
        .xmit     =
        {
            .size   = CONFIG_UART7_TXBUFSIZE,
            .buffer = g_uart7txbuffer,
        },
        .ops      = &g_uart_ops,
        .priv     = &g_uart7priv,
    },

    .irq            = STM32_IRQ_UART7,
    .parity         = CONFIG_UART7_PARITY,
    .bits           = CONFIG_UART7_BITS,
    .stopbits2      = CONFIG_UART7_2STOP,
    .baud           = CONFIG_UART7_BAUD,
    .apbclock       = STM32_PCLK1_FREQUENCY,
    .usartbase      = STM32_UART7_BASE,
    .tx_gpio        = GPIO_UART7_TX,
    .rx_gpio        = GPIO_UART7_RX,
#if defined(CONFIG_SERIAL_OFLOWCONTROL) && defined(CONFIG_UART7_OFLOWCONTROL)
    .oflow          = true,
    .cts_gpio       = GPIO_UART7_CTS,
#endif
#if defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_UART7_IFLOWCONTROL)
    .iflow          = true,
    .rts_gpio       = GPIO_UART7_RTS,
#endif
};
#endif

/* This describes the state of the STM32 UART8 port. */

#if defined(CONFIG_STM32_UART8)
static struct up_dev_s g_uart8priv =
{
    .dev =
    {
#if CONSOLE_UART == 8
        .isconsole = true,
#endif
        .recv     =
        {
            .size   = CONFIG_UART8_RXBUFSIZE,
            .buffer = g_uart8rxbuffer,
        },
        .xmit     =
        {
            .size   = CONFIG_UART8_TXBUFSIZE,
            .buffer = g_uart8txbuffer,
        },
        .ops      = &g_uart_ops,
        .priv     = &g_uart8priv,
    },

    .irq            = STM32_IRQ_UART8,
    .parity         = CONFIG_UART8_PARITY,
    .bits           = CONFIG_UART8_BITS,
    .stopbits2      = CONFIG_UART8_2STOP,
    .baud           = CONFIG_UART8_BAUD,
    .apbclock       = STM32_PCLK1_FREQUENCY,
    .usartbase      = STM32_UART8_BASE,
    .tx_gpio        = GPIO_UART8_TX,
    .rx_gpio        = GPIO_UART8_RX,
#if defined(CONFIG_SERIAL_OFLOWCONTROL) && defined(CONFIG_UART8_OFLOWCONTROL)
    .oflow          = true,
    .cts_gpio       = GPIO_UART8_CTS,
#endif
#if defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_UART8_IFLOWCONTROL)
    .iflow          = true,
    .rts_gpio       = GPIO_UART8_RTS,
#endif
};
#endif

/* This table lets us iterate over the configured USARTs */

static struct up_dev_s * const uart_devs[STM32_NUSART] =
{
#if defined(CONFIG_STM32_USART1)
    [0] = &g_usart1priv,
#else
    [0] = 0,
#endif
#if defined(CONFIG_STM32_USART2)
    [1] = &g_usart2priv,
#else
    [1] = 0,
#endif
#if defined(CONFIG_STM32_USART3)
    [2] = &g_usart3priv,
#else
    [2] = 0,
#endif
#if defined(CONFIG_STM32_UART4)
    [3] = &g_uart4priv,
#else
    [3] = 0,
#endif
#if defined(CONFIG_STM32_UART5)
    [4] = &g_uart5priv,
#else
    [4] = 0,
#endif
#if defined(CONFIG_STM32_USART6)
    [5] = &g_usart6priv,
#else
    [5] = 0,
#endif
#if defined(CONFIG_STM32_UART7)
    [6] = &g_uart7priv,
#else
    [6] = 0,
#endif
#if defined(CONFIG_STM32_UART8)
    [7] = &g_uart8priv,
#else
    [7] = 0,
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline uint32_t uart_getreg8(struct up_dev_s *priv, int offset)
{
    return getreg8(priv->usartbase + offset);
}

static inline void uart_putreg8(struct up_dev_s *priv, int offset,
        uint32_t value)
{
    putreg8(value, priv->usartbase + offset);
}

static inline uint32_t uart_getreg16(struct up_dev_s *priv, int offset)
{
    return getreg16(priv->usartbase + offset);
}

static inline void uart_putreg16(struct up_dev_s *priv, int offset,
        uint32_t value)
{
    putreg16(value, priv->usartbase + offset);
}

static inline uint32_t uart_getreg32(struct up_dev_s *priv, int offset)
{
    return getreg32(priv->usartbase + offset);
}

static inline void uart_putreg32(struct up_dev_s *priv, int offset,
        uint32_t value)
{
    putreg32(value, priv->usartbase + offset);
}

/****************************************************************************
 * Name: up_restoreusartint
 ****************************************************************************/

static void up_restoreusartint(struct up_dev_s *priv, uint16_t ie)
{
    uint32_t cr;

    /* Save the interrupt mask */

    priv->ie = ie;

    /* And restore the interrupt state (see the interrupt enable/usage table above) */

    cr = uart_getreg32(priv, STM32_USART_CR1_OFFSET);
    cr &= ~(USART_CR1_USED_INTS);
    cr |= (ie & (USART_CR1_USED_INTS));
    uart_putreg32(priv, STM32_USART_CR1_OFFSET, cr);

    cr = uart_getreg32(priv, STM32_USART_CR3_OFFSET);
    cr &= ~USART_CR3_EIE;
    cr |= (ie & USART_CR3_EIE);
    uart_putreg32(priv, STM32_USART_CR3_OFFSET, cr);
}

/****************************************************************************
 * Name: up_disableusartint
 ****************************************************************************/

static inline void up_disableusartint(struct up_dev_s *priv, uint16_t *ie)
{
    if (ie)
    {
        uint32_t cr1;
        uint32_t cr3;

        /* USART interrupts:
         *
         * Enable             Status          Meaning                        Usage
         * ------------------ --------------- ------------------------------ ----------
         * USART_CR1_IDLEIE   USART_SR_IDLE   Idle Line Detected             (not used)
         * USART_CR1_RXNEIE   USART_SR_RXNE   Received Data Ready to be Read
         * "              "   USART_SR_ORE    Overrun Error Detected
         * USART_CR1_TCIE     USART_SR_TC     Transmission Complete          (used only for RS-485)
         * USART_CR1_TXEIE    USART_SR_TXE    Transmit Data Register Empty
         * USART_CR1_PEIE     USART_SR_PE     Parity Error
         *
         * USART_CR2_LBDIE    USART_SR_LBD    Break Flag                     (not used)
         * USART_CR3_EIE      USART_SR_FE     Framing Error
         * "           "      USART_SR_NE     Noise Error
         * "           "      USART_SR_ORE    Overrun Error Detected
         * USART_CR3_CTSIE    USART_SR_CTS    CTS flag                       (not used)
         */

        cr1 = uart_getreg32(priv, STM32_USART_CR1_OFFSET);
        cr3 = uart_getreg32(priv, STM32_USART_CR3_OFFSET);

        /* Return the current interrupt mask value for the used interrupts.  Notice
         * that this depends on the fact that none of the used interrupt enable bits
         * overlap.  This logic would fail if we needed the break interrupt!
         */

        *ie = (cr1 & (USART_CR1_USED_INTS)) | (cr3 & USART_CR3_EIE);
    }

    /* Disable all interrupts */

    up_restoreusartint(priv, 0);
}

/****************************************************************************
 * Name: up_set_format
 *
 * Description:
 *   Set the serial line format and speed.
 *
 ****************************************************************************/

#ifndef CONFIG_SUPPRESS_UART_CONFIG
static void up_set_format(struct uart_dev_s *dev)
{
    struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
    uint32_t usartdiv32;
    uint32_t mantissa;
    uint32_t fraction;
    uint32_t regval;
    uint32_t brr;

    /* Load CR1 */

    regval = uart_getreg32(priv, STM32_USART_CR1_OFFSET);

    /* This implementation is for U[S]ARTs that support fractional
     * dividers.
     *
     * Configure the USART Baud Rate.  The baud rate for the receiver and
     * transmitter (Rx and Tx) are both set to the same value as programmed
     * in the Mantissa and Fraction values of USARTDIV.
     *
     *   baud     = fCK / (16 * usartdiv)
     *   usartdiv = fCK / (16 * baud)
     *
     * Where fCK is the input clock to the peripheral (PCLK1 for USART2, 3, 4, 5
     * or PCLK2 for USART1)
     *
     * First calculate (NOTE: all stand baud values are even so dividing by two
     * does not lose precision):
     *
     *   usartdiv32 = 32 * usartdiv = fCK / (baud/2)
     */

    usartdiv32 = priv->apbclock / (priv->baud >> 1);

    /* The mantissa part is then */

    mantissa   = usartdiv32 >> 5;

    /* The fractional remainder (with rounding) */

    fraction   = (usartdiv32 - (mantissa << 5) + 1) >> 1;

    /* The F4 supports 8 X in oversampling additional to the
     * standard oversampling by 16.
     *
     * With baud rate of fCK / (16 * Divider) for oversampling by 16.
     * and baud rate of  fCK /  (8 * Divider) for oversampling by 8
     */

    /* Check if 8x oversampling is necessary */

    if (mantissa == 0)
    {
        regval |= USART_CR1_OVER8;

        /* Rescale the mantissa */

        mantissa = usartdiv32 >> 4;

        /* The fractional remainder (with rounding) */

        fraction = (usartdiv32 - (mantissa << 4) + 1) >> 1;
    }
    else
    {
        /* Use 16x Oversampling */

        regval &= ~USART_CR1_OVER8;
    }

    brr  = mantissa << USART_BRR_MANT_SHIFT;
    brr |= fraction << USART_BRR_FRAC_SHIFT;

    uart_putreg32(priv, STM32_USART_CR1_OFFSET, regval);
    uart_putreg32(priv, STM32_USART_BRR_OFFSET, brr);

    /* Configure parity mode */

    regval &= ~(USART_CR1_PCE | USART_CR1_PS | USART_CR1_M);

    if (priv->parity == 1)       /* Odd parity */
    {
        regval |= (USART_CR1_PCE | USART_CR1_PS);
    }
    else if (priv->parity == 2)  /* Even parity */
    {
        regval |= USART_CR1_PCE;
    }

    /* Configure word length (parity uses one of configured bits)
     *
     * Default: 1 start, 8 data (no parity), n stop, OR
     *          1 start, 7 data + parity, n stop
     */

    if (priv->bits == 9 || (priv->bits == 8 && priv->parity != 0))
    {
        /* Select: 1 start, 8 data + parity, n stop, OR
         *         1 start, 9 data (no parity), n stop.
         */

        regval |= USART_CR1_M;
    }

    uart_putreg32(priv, STM32_USART_CR1_OFFSET, regval);

    /* Configure STOP bits */

    regval = uart_getreg32(priv, STM32_USART_CR2_OFFSET);
    regval &= ~(USART_CR2_STOP_MASK);

    if (priv->stopbits2)
    {
        regval |= USART_CR2_STOP2;
    }

    uart_putreg32(priv, STM32_USART_CR2_OFFSET, regval);

    /* Configure hardware flow control */

    regval  = uart_getreg32(priv, STM32_USART_CR3_OFFSET);
    regval &= ~(USART_CR3_CTSE | USART_CR3_RTSE);

#if defined(CONFIG_SERIAL_IFLOWCONTROL) && \
    !defined(CONFIG_STM32_FLOWCONTROL_BROKEN)
    if (priv->iflow && (priv->rts_gpio != 0))
    {
        regval |= USART_CR3_RTSE;
    }
#endif

#ifdef CONFIG_SERIAL_OFLOWCONTROL
    if (priv->oflow && (priv->cts_gpio != 0))
    {
        regval |= USART_CR3_CTSE;
    }
#endif

    uart_putreg32(priv, STM32_USART_CR3_OFFSET, regval);
}
#endif /* CONFIG_SUPPRESS_UART_CONFIG */

/****************************************************************************
 * Name: up_set_apb_clock
 *
 * Description:
 *   Enable or disable APB clock for the USART peripheral
 *
 * Input parameters:
 *   dev - A reference to the UART driver state structure
 *   on  - Enable clock if 'on' is 'true' and disable if 'false'
 *
 ****************************************************************************/

static void up_set_apb_clock(struct uart_dev_s *dev, bool on)
{
    struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
    uint32_t rcc_en;
    uint32_t regaddr;

    /* Determine which USART to configure */

    switch (priv->usartbase)
    {
        default:
            return;
        case STM32_USART1_BASE:
            rcc_en = RCC_APB2ENR_USART1EN;
            regaddr = STM32_RCC_APB2ENR;
            break;
        case STM32_USART2_BASE:
            rcc_en = RCC_APB1ENR_USART2EN;
            regaddr = STM32_RCC_APB1ENR;
            break;
        case STM32_USART3_BASE:
            rcc_en = RCC_APB1ENR_USART3EN;
            regaddr = STM32_RCC_APB1ENR;
            break;
        case STM32_UART4_BASE:
            rcc_en = RCC_APB1ENR_UART4EN;
            regaddr = STM32_RCC_APB1ENR;
            break;
        case STM32_UART5_BASE:
            rcc_en = RCC_APB1ENR_UART5EN;
            regaddr = STM32_RCC_APB1ENR;
            break;
        case STM32_USART6_BASE:
            rcc_en = RCC_APB2ENR_USART6EN;
            regaddr = STM32_RCC_APB2ENR;
            break;
        case STM32_UART7_BASE:
            rcc_en = RCC_APB1ENR_UART7EN;
            regaddr = STM32_RCC_APB1ENR;
            break;
        case STM32_UART8_BASE:
            rcc_en = RCC_APB1ENR_UART8EN;
            regaddr = STM32_RCC_APB1ENR;
            break;
    }

    /* Enable/disable APB 1/2 clock for USART */

    if (on)
    {
        modifyreg32(regaddr, 0, rcc_en);
    }
    else
    {
        modifyreg32(regaddr, rcc_en, 0);
    }
}

