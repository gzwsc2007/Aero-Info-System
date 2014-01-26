/* 
 * GPS.c
 * Author: Anson Wang
 *
 * This module provides interface for accessing information from the GPS sensor.
 * All interpreted data is contained in the global structure GPS_Data.
 * GPS data is read from the serial port, and transfered to main memory by DMA.
 * Then, in the USART_IRQHandler, raw GPS data is parsed and stored into GPS_Data.
 *
 */

#include "stm32f4xx.h"
#include <rtthread.h>
#include <string.h>
#include "MAVlink_include/common/mavlink.h"
#include "GPS.h"
#include <inttypes.h>
#include "Telemetry.h"

/* Gloabl Variables Definitions */
rt_bool_t GPS_struct_busy = RT_FALSE;  // indicates whether the GPS data is being accessed
GPS_Data_t GPS_Data;  // the GPS data structure that stores interpreted GPS data.

/* Private Variables */
static char GPS_buf[GPS_BUF_LEN];  // buffer for receiving NMEA 0183 formatted data

/* Function Prototypes */
static void GPRMC_interp_block(char* b, rt_int8_t i, GPS_Data_t* g);
static void GPS_parse(char* s, GPS_Data_t* g);
static rt_int32_t NMEA_atoi(const char* s);
static rt_int32_t NMEA_convertLatLong(char *b);


/* 
 * Interrupt is triggered when idle line is detected.
 * Note that the GPS receiver sends data every 1 second, so we will parse
 * the NMEA string every 1 second.
 */
void USART1_IRQHandler()
{
    if (USART_GetITStatus(GPS_USART, USART_IT_IDLE) != RESET)
    {
        rt_uint32_t temp;
        // A sequence of read from SR and DR clears the interrupt bit
        temp = GPS_USART->SR;
        temp = GPS_USART->DR;

        // Temporarily diasble DMA so that we can safely access GPS_buf
        DMA_Cmd(DMA2_Stream5, DISABLE);
        // Obtain the valid length of the GPS_buf
        temp = GPS_BUF_LEN - DMA_GetCurrDataCounter(DMA2_Stream5);

        // Parse the GPS data only if the GPS_Data struct is not being accessed.
        if (!GPS_struct_busy)
        {
            GPS_buf[temp] = '\0';
            GPS_parse(GPS_buf, &GPS_Data); 
            rt_event_send(&event_drdy, EVENT_GPS_DATA_RDY);
        }
        
        // Reset the DMA data counter
        DMA_SetCurrDataCounter(DMA2_Stream5, GPS_BUF_LEN);
        // Re-enable DMA
        DMA_Cmd(DMA2_Stream5, ENABLE);
    }
}


void GPS_fillMavlinkStruct(mavlink_gps_raw_int_t *m)
{
    if (GPS_struct_busy)
        return;
    else 
        GPS_struct_busy = RT_TRUE;
    
    if (!GPS_Data.valid) 
    {
        m->fix_type = 0; // for me, 0: invalid data
        GPS_struct_busy = RT_FALSE;
        return;
    }
    
    m->time_usec = (uint64_t)rt_tick_get() * (uint64_t)1000;
    m->fix_type = 1; // 0-1: no fix. 
    m->lat = GPS_Data.latitude;
    m->lon = GPS_Data.longitude;
    m->alt = 0; // gps alt not known
    m->eph = 0xFFFF;
    m->epv = 0xFFFF;
    m->vel = GPS_Data.speed;
    m->cog = GPS_Data.course;
    m->satellites_visible = 0xFF; // unknown
    
    GPS_struct_busy = RT_FALSE;
}


/*
 * Take an NMEA string, parse it, and store the interpreted information in the 
 * GPS_Data structure.
 * 
 * Supports GPRMC for now.
 */
static void GPS_parse(char* s, GPS_Data_t* g)
{
    // a block for interpretation
    char* block;
    rt_int8_t index=1;  // block index
    
    // determine the format of the string (can be used to extend to other format)
    if (strncmp(s, "$GPRMC", 6)) 
    {
        rt_kprintf("Unsupported string\n");
        return;
    }
    
    s += 7;
    block = s;
    while (*s != '\0')
    {
        if (*s == ',')
        {
            *s = '\0';  // set the "end" of a block
            GPRMC_interp_block(block, index, g);
            s++;
            index++;
            block = s;
            continue;
        }
        s++;
    }
}


/* Helper function to parse GPRMC string */
static void GPRMC_interp_block(char* b, rt_int8_t i, GPS_Data_t* g)
{
    rt_int32_t temp;
    
    if (*b == '\0') return;
    
    switch(i) 
    {
        // Navigation Status: A = valid, V = invalid
        case 2: 
            if (*b == 'A') g->valid = RT_TRUE;
            else g->valid = RT_FALSE;
            break;
        // Latitude
        case 3:
            g->latitude = NMEA_convertLatLong(b);
            break;
        // N or S Hemisphere
        case 4:
            if (*b == 'S') g->latitude = -(g->latitude);  // Use negative sign to denote S
            break;
        // Longitude
        case 5:
            g->longitude = NMEA_convertLatLong(b);
            break;
        // E or W Hemisphere
        case 6:
            if (*b == 'W') g->longitude = -(g->longitude);
            break;
        // Ground speed
        case 7:
            temp = NMEA_atoi(b);
            g->speed = (rt_uint16_t)(temp * 643 / 125); // convert 1 knot to 0.01 m/s
            break;
        // Course
        case 8:
            g->course = (rt_uint16_t)NMEA_atoi(b) * 10;
            break;
    }
}


/* 
 * Helper function to convert raw GPRMC lat/long string to an integer
 * with unit 10^-7 degree.
 */
static rt_int32_t NMEA_convertLatLong(char *b) {
    char *tmp = strchr(b, '.'); // locate the '.' char
    double min;
    int deg;
    int result;
     
    // convert minute into the form of mm.mmmm
    tmp = tmp - 2; // go back to the first 'm'
    min = (double)NMEA_atoi(tmp);
    
    // convert degree into a single integer
    *tmp = '\0'; // safe to modify b in this case
    deg = NMEA_atoi(b);
    
    // scaling
    result = deg * 10000000; // scale 1 deg to 10^-7 deg
    min = (min / 600000.0) * 10000000.0; // convert minute to 10^-7 deg
    
    result = result + (rt_int32_t)min;
    return result;
}


/*
 * Convert strings of the format "ddmm.mmmm" into integer.
 * Borrowed from 
 * https://github.com/offchooffcho/STM32-1/blob/master/GPSTracker/car/nmea.c
 */
static rt_int32_t NMEA_atoi(const char* s)
{
    rt_int32_t result = 0;

    while ((*s >= '0' && *s <= '9') || *s == '.')
    {
        if (*s == '.') 
        {
            s++;
            continue;
        }
        result *= 10;
        result += *s - '0';
        s++;
    }

    return result;
}


static void USART_Configuration() 
{
    USART_InitTypeDef USART_InitStructure;
    
    // Initialize the USART that is used to receive GPS data.
    USART_DeInit(GPS_USART);
    USART_InitStructure.USART_BaudRate = GPS_USART_BAUD_RATE;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;  // No Parity check
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx;  // For receiving only
    
    USART_Init(GPS_USART, &USART_InitStructure);
    
    // detect idle line and cause an interrupt in which the GPS data is parsed
    USART_ITConfig(GPS_USART, USART_IT_IDLE, ENABLE);
    USART_ITConfig(GPS_USART, USART_IT_RXNE, DISABLE);
    
    USART_Cmd(GPS_USART, ENABLE);
}


static void DMA_Configuration()
{
	//串口收DMA配置 USART1_RX -> DMA2, Channel 4, Stream 5
    DMA_InitTypeDef DMA_InitStructure;
    DMA_DeInit(DMA2_Stream5); 
    
	//启动DMA时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
    DMA_InitStructure.DMA_Channel = DMA_Channel_4;
	//外设地址
	DMA_InitStructure.DMA_PeripheralBaseAddr = (rt_uint32_t)(&GPS_USART->DR);
	//内存地址
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)GPS_buf;
	//dma传输方向单向
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	//设置DMA在传输时缓冲区的长度
	DMA_InitStructure.DMA_BufferSize = GPS_BUF_LEN;
	//设置DMA的外设递增模式禁用，一个外设
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	//设置DMA的内存递增模式
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	//外设数据字长
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	//内存数据字长
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	//设置DMA的传输模式
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;  // has to be circular mode+
    
	//设置DMA的优先级别
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    // Disable FIFO Mode. Disable Burst mode.
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;  
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    
    
	DMA_Init(DMA2_Stream5,&DMA_InitStructure);
	//使能通道5
	DMA_Cmd(DMA2_Stream5,ENABLE);  
    // Enable DMA for GPS_USART
    USART_DMACmd(GPS_USART, USART_DMAReq_Rx, ENABLE);
}


/*
 * Called by rt_application_init() in application.c
 */
void GPS_Init()
{
    USART_Configuration();
    DMA_Configuration();
}
