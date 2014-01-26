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
#include "I2CSensors.h"
#include "MPU6050.h"
#include "Telemetry.h"

// stack size for the mavlink thread
#define MAVLINK_THREAD_STACK_SIZE     2048
// priority for the radio_thread
#define MAVLINK_THREAD_PRIORITY       16

// stack size for the BMP085_thread
#define BMP085_THREAD_STACK_SIZE     512
// priority for the BMP085_thread
#define BMP085_THREAD_PRIORITY       16

// stack size for the ADS1115 thread
#define ADS1115_THREAD_STACK_SIZE    512
// priority for the ADS1115 thread
#define ADS1115_THREAD_PRIORITY      16
     
static rt_uint8_t mavlink_thread_stack[MAVLINK_THREAD_STACK_SIZE]; 
static struct rt_thread mavlink_thread;

static rt_uint8_t BMP085_thread_stack[BMP085_THREAD_STACK_SIZE]; 
static struct rt_thread BMP085_thread;

static rt_uint8_t ADS1115_thread_stack[ADS1115_THREAD_STACK_SIZE];
static struct rt_thread ADS1115_thread;


int rt_application_init()
{
    GPS_Init();
    NRF24_Init();
    I2CSensors_Init();
    MPU6050_Init();
    
    // start the BMP085_thread
    rt_thread_init(
                   &BMP085_thread, 
                   "BMP085", 
                   BMP085_thread_entry, 
                   RT_NULL, 
                   BMP085_thread_stack,
                   BMP085_THREAD_STACK_SIZE, 
                   BMP085_THREAD_PRIORITY, 
                   10
                  );
    rt_thread_startup(&BMP085_thread);
    
    // start the ADS1115 thread
   /* rt_thread_init(
                   &ADS1115_thread, 
                   "ADS1115", 
                   ADS1115_thread_entry, 
                   RT_NULL, 
                   ADS1115_thread_stack,
                   ADS1115_THREAD_STACK_SIZE, 
                   ADS1115_THREAD_PRIORITY, 
                   10
                  );
    rt_thread_startup(&ADS1115_thread);*/
 
    // start the mavlink_thread
    rt_thread_init(
                   &mavlink_thread, 
                   "mavlink", 
                   mavlink_thread_entry, 
                   RT_NULL, 
                   mavlink_thread_stack,
                   MAVLINK_THREAD_STACK_SIZE, 
                   MAVLINK_THREAD_PRIORITY, 
                   10
                  );
    rt_thread_startup(&mavlink_thread);
    
    return 0;
}

/*@}*/
