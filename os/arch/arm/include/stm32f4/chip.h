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
