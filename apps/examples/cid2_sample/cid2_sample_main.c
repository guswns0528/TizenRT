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
 * examples/cid2_sample/cid2_sample_main.c
 *
 *   Copyright (C) 2008, 2011-2012 Gregory Nutt. All rights reserved.
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
#include <tinyara/gpio.h>
#include <tinyara/ascii.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/select.h>
#include <apps/shell/tash.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/
#define TASK_PRI        100
#define TASK_STACKSIZE  1024

/****************************************************************************
 * Private Data & Functions
 ****************************************************************************/

#define GREEN 0
#define YELLOW 1
#define RED 2

int fd_r = -1;
int fd_y = -1;
int fd_g = -1;

int *leds[] = {
    &fd_g,
    &fd_y,
    &fd_r
};

static void turnon_led(int color)
{
    if (color < GREEN || color > RED)
        return;
    write(*leds[color], "1", 1);
}
static void turnoff_led(int color)
{
    if (color < GREEN || color > RED)
        return;
    write(*leds[color], "0", 1);
}
static void init_leds(void)
{
    fd_r = open("/dev/gpio0", O_WRONLY);
    if (fd_r >= 0)
        ioctl(fd_r, GPIOIOC_SET_DIRECTION, GPIO_DIRECTION_OUT);
    fd_y = open("/dev/gpio1", O_WRONLY);
    if (fd_y >= 0)
        ioctl(fd_y, GPIOIOC_SET_DIRECTION, GPIO_DIRECTION_OUT);
    fd_g = open("/dev/gpio2", O_WRONLY);
    if (fd_g >= 0)
        ioctl(fd_g, GPIOIOC_SET_DIRECTION, GPIO_DIRECTION_OUT);
}
static void close_leds(void)
{
    if (fd_r >= 0)
    {
        turnoff_led(RED);
        close(fd_r);
    }
    if (fd_y >= 0)
    {
        turnoff_led(YELLOW);
        close(fd_y);
    }
    if (fd_g >= 0)
    {
        turnoff_led(GREEN);
        close(fd_g);
    }
}

static int next_state(int state)
{
    if (state == GREEN)
    {
        return YELLOW;
    }
    else if (state == YELLOW)
    {
        return RED;
    }
    else if (state == RED)
    {
        return GREEN;
    }

    return GREEN;
}

static void led_toggle(void)
{
    int state = GREEN;
    int loop_count = 12;
    init_leds();
    if (fd_r < 0 || fd_y < 0 || fd_g < 0)
    {
        printf("fail to open some led device\n");
        goto end;
    }

    state = GREEN;
    while (loop_count--)
    {
        turnon_led(state);
        sleep(1);
        turnoff_led(state);
        state = next_state(state);
    }
end:
    close_leds();
}

#define SELECT_TIMEOUT_SECS (6)
#define SELECT_TIMEOUT_USECS (0)
static char* read_line(int fd)
{
    int bufsize = 128;
    int pos = 0;
    int nbytes = 0;
    int char_idx = 0;
#if !defined(CONFIG_DISABLE_POLL)
    fd_set tfd;
    struct timeval stimeout;
    stimeout.tv_sec = SELECT_TIMEOUT_SECS;
    stimeout.tv_usec = SELECT_TIMEOUT_USECS;
#endif
    char *buffer = malloc(sizeof(char) * bufsize);

    if (!buffer) {
        return NULL;
    }

    memset(buffer, 0x0, bufsize);

    do {
#if !defined(CONFIG_DISABLE_POLL)
        FD_ZERO(&tfd);
        FD_SET(fd, &tfd);

        if ((select(fd + 1, &tfd, NULL, NULL, &stimeout)) && FD_ISSET(fd, &tfd)) {
#endif
            /* read characters */
            nbytes = read(fd, &buffer[pos], (bufsize - pos));
            if (nbytes < 0)
            {
                free(buffer);
                return NULL;
            }

            for (char_idx = 0; char_idx < nbytes; char_idx++)
            {
                /* treat backspace and delete */
                if ((buffer[pos] == ASCII_BS) || (buffer[pos] == ASCII_DEL))
                {
                    int valid_char_pos = pos + 1;
                    if (pos > 0)
                    {
                        pos--;
                        /* Update screen */
                        write(fd, "\b \b", 3);
                    }

                    if ((buffer[valid_char_pos] != 0x0) && (valid_char_pos < bufsize))
                    {
                        memmove(&buffer[pos], &buffer[valid_char_pos], (bufsize - valid_char_pos));
                    }
                }
                else
                {
                    if (buffer[pos] == ASCII_CR)
                    {
                        buffer[pos] = ASCII_LF;
                    }

                    /* echo */
                    write(fd, &buffer[pos], 1);
                    pos++;
                    if (pos >= bufsize)
                    {
                        buffer[bufsize - 1] = ASCII_NUL;
                        return buffer;
                    }
                }
            }
#if !defined(CONFIG_DISABLE_POLL)
        }
#endif
    }
    while (buffer[pos - 1] != ASCII_LF);

    buffer[pos - 1] = ASCII_NUL;
    return buffer;
}

static void echo(void)
{
    while (1)
    {
        char* buf = read_line(fileno(stdin));
        if (buf == NULL || strcmp(buf, "quit") == 0)
        {
            free(buf);
            break;
        }
        printf("%s\n", buf);
        free(buf);
    }
}

static int str_to_color(char* p)
{
    if (strcmp(p, "red") == 0)
        return RED;
    if (strcmp(p, "yellow") == 0)
        return YELLOW;
    if (strcmp(p, "green") == 0)
        return GREEN;

    return -1;
}

#define DELIM " \r\n\t"

static void led_command(void)
{
    init_leds();
    if (fd_r < 0 || fd_y < 0 || fd_g < 0)
    {
        printf("fail to open some led device\n");
        goto end;
    }

    while (1)
    {
        char* command = read_line(fileno(stdin));
        char* token = NULL;
        token = strtok(command, DELIM);
        if (command == NULL || strcmp(token, "quit") == 0)
        {
            free(command);
            break;
        }

        if (strcmp(command, "on") == 0)
        {
            int color;
            token = strtok(NULL, DELIM);
            color = str_to_color(token);
            if (color == -1)
                printf("invalid color\n");
            else
                turnon_led(color);
        }
        else if (strcmp(command, "off") == 0)
        {
            int color;
            token = strtok(NULL, DELIM);
            color = str_to_color(token);
            if (color == -1)
                printf("invalid color\n");
            else
                turnoff_led(color);
        }
        else
        {
            printf("invalid command\n");
        }
        free(command);
    }
end:
    close_leds();
}

static int do_cid2_sample(char *argv[])
{
    int i = 0;
    for (i = 0; argv[i]; i++)
    {
        if (i == 1 && argv[i])
        {
            if (strcmp(argv[i], "led_toggle") == 0)
                led_toggle();
            else if (strcmp(argv[i], "echo") == 0)
                echo();
            else if (strcmp(argv[i], "led_command") == 0)
                led_command();
        }
    }

    return 0;
}

static void cid2_sample_cb(int argc, char **args)
{
    pthread_t cid2_sample;
    pthread_attr_t attr;
    struct sched_param sparam;
    int status;
#ifdef SDCC
    pthread_addr_t result;
#endif

    /* Initialize the attribute variable */
    status = pthread_attr_init(&attr);
    if (status != 0) {
        printf("cid2_sample : pthread_attr_init failed, status=%d\n", status);
    }

    /* 1. set a priority */
    sparam.sched_priority = TASK_PRI;
    status = pthread_attr_setschedparam(&attr, &sparam);
    if (status != OK) {
        printf("cid2_sample : pthread_attr_setschedparam failed, status=%d\n", status);
    }

    /* 2. set a stacksize */
    status = pthread_attr_setstacksize(&attr, TASK_STACKSIZE);
    if (status != OK) {
        printf("cid2_sample : pthread_attr_setstacksize failed, status=%d\n", status);
    }

    /* 3. create pthread with entry function */
    status = pthread_create(&cid2_sample, &attr, do_cid2_sample, (void *)args);
    if (status != 0) {
        printf("cid2_sample: pthread_create failed, status=%d\n", status);
    }

    /* Wait for the threads to stop */
#ifdef SDCC
    pthread_join(cid2_sample, &result);
#else
    pthread_join(cid2_sample, NULL);
#endif
}


/****************************************************************************
 * cid2_sample_main
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int cid2_sample_main(int argc, char *argv[])
#endif
{
    tash_cmd_install("cid2", cid2_sample_cb, TASH_EXECMD_SYNC);
    return 0;
}
