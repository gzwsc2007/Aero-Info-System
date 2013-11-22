/*
 * File      : application.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      the first version
 */

/**
 * @addtogroup STM32
 */
/*@{*/

#include <stdio.h>

#include "stm32f4xx.h"
#include <board.h>
#include <rtthread.h>
#include "GPS.h"
#include "NRF24L01.h"

// stack size for the radio_thread
#define RADIO_THREAD_STACK_SIZE     256
// priority for the radio_thread
#define RADIO_THREAD_PRIORITY       16

static rt_uint8_t thread_stack[RADIO_THREAD_STACK_SIZE]; 
static struct rt_thread radio_thread;

int rt_application_init()
{
    GPS_Init();
    NRF24_Init();
    
    // start the radio_thread
    rt_thread_init(&radio_thread, "radio", radio_thread_entry, RT_NULL, thread_stack,
                   RADIO_THREAD_STACK_SIZE, RADIO_THREAD_PRIORITY, 10);
    rt_thread_startup(&radio_thread);
    
    return 0;
}

/*@}*/
