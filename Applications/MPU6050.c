#include "stm32f4xx.h"
#include <rtthread.h>
#include "MAVlink_include/aeroInfoSystem/mavlink.h"
#include "MPU6050.h"
#include "Telemetry.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define DEG_TO_RAD  (M_PI / 180.0)

MPU_Data_t MPU_Data;
rt_bool_t MPU_struct_busy = RT_FALSE;

static rt_uint8_t rxbuf[MPU_BUF_LEN];

static rt_int8_t parse(rt_uint8_t *buf, MPU_Data_t *m);

/*
 * Interrupt is triggered when an idle line is detected.
 */
void USART3_IRQHandler()
{
    if (USART_GetITStatus(MPU_USART, USART_IT_IDLE) != RESET)
    {
        rt_uint32_t temp;
        static rt_uint8_t cnt = 0;
        
        // A sequence of read from SR and DR clears the interrupt bit
        temp = MPU_USART->SR;
        temp = MPU_USART->DR;
        
        // Temporarily diasble DMA so that we can safely access rxbuf
        DMA_Cmd(DMA1_Stream1, DISABLE);
        // Obtain the valid length of the rxbuf
        temp = MPU_BUF_LEN - DMA_GetCurrDataCounter(DMA1_Stream1);
        
        cnt++;
        
        // parse data here
        if (cnt >= 3)
        {
            cnt = 0;
            if (!MPU_struct_busy)
            {
                MPU_struct_busy = RT_TRUE;
                if (temp > 33 || (parse(rxbuf, &MPU_Data) == -1))
                    memset(&MPU_Data, 0, sizeof(MPU_Data)); // deal with error
                MPU_struct_busy = RT_FALSE;
                rt_event_send(&event_drdy, EVENT_ATTITUDE_DATA_RDY);
            }
        }
        
        // Reset the DMA data counter
        DMA_SetCurrDataCounter(DMA1_Stream1, MPU_BUF_LEN);
        // Re-enable DMA
        DMA_Cmd(DMA1_Stream1, ENABLE);
    }
}


static rt_int8_t parse(rt_uint8_t *buf, MPU_Data_t *m)
{
    // skip the accel packet
    rt_uint32_t ptr = 11;
    
    // decode angular vel and angle packet
    if (buf[ptr++] == 0x55)
    {
        // angular vel packet
        if (buf[ptr++] == 0x52)
        {
            m->rollspeed = ((rt_int16_t)(buf[ptr+1]<<8| buf[ptr]))/32768.0*2000.0;
            ptr += 2;
            m->pitchspeed = ((rt_int16_t)(buf[ptr+1]<<8| buf[ptr]))/32768.0*2000.0;
            ptr += 2;
            m->yawspeed = ((rt_int16_t)(buf[ptr+1]<<8| buf[ptr]))/32768.0*2000.0;
            ptr += 2;
            m->temperature = ((rt_int16_t)(buf[ptr+1]<<8| buf[ptr]))/340.0+36.25;
            ptr += 3;
        }
        else 
            return -1;
        
        if (buf[ptr++] != 0x55)
            return -1;
        
        // angle packet
        if (buf[ptr++] == 0x53)
        {
            m->roll = ((rt_int16_t)(buf[ptr+1]<<8| buf[ptr]))/32768.0*180;
            ptr += 2;
            m->pitch = ((rt_int16_t)(buf[ptr+1]<<8| buf[ptr]))/32768.0*180;
            ptr += 2;
            m->yaw = ((rt_int16_t)(buf[ptr+1]<<8| buf[ptr]))/32768.0*180;
            ptr += 2;
            m->temperature = ((rt_int16_t)(buf[ptr+1]<<8| buf[ptr]))/340.0+36.25;
            ptr += 3;
        }
        else
            return -1;
    }
    else 
        return -1;
    
    return 0;
}


void MPU6050_Init(void)
{
    USART_InitTypeDef USART_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;
    GPIO_InitTypeDef GPIO_InitStruct;
    NVIC_InitTypeDef NVIC_InitStructure;
            
    /* RCC Configuration */
    // enabl USART3 clock
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    // enable DMA1 clock
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
    // enable GPIOC clock
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    
    /* GPIO Configuration */
    
    // configure pins used by USART3
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStruct);
    
    // connect pins to USART3 alternate function
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_USART3);
    
    /* USART3 Configuration */
    
    // Initialize the USART that is used to receive MPU data.
    USART_DeInit(MPU_USART);
    USART_InitStructure.USART_BaudRate = MPU_USART_BAUD_RATE;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;  // No Parity check
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(MPU_USART, &USART_InitStructure);
    
    // detect idle line and cause an interrupt in which the MPU data is parsed
    USART_ITConfig(MPU_USART, USART_IT_IDLE, ENABLE);
    USART_ITConfig(MPU_USART, USART_IT_RXNE, DISABLE);
    
    USART_Cmd(MPU_USART, ENABLE);
    
    /* make sure the Module is in Serial mode (not I2C mode)
    while(USART_GetFlagStatus(MPU_USART, USART_FLAG_TXE) == RESET);
    USART_SendData(MPU_USART, 0x61);
    while(USART_GetFlagStatus(MPU_USART, USART_FLAG_TXE) == RESET);
    USART_SendData(MPU_USART, 0x63);
    while(USART_GetFlagStatus(MPU_USART, USART_FLAG_TXE) == RESET);*/
               
    
    /* DMA Configuration */
    
    //串口收DMA配置 USART3_RX -> DMA1, Channel 4, Stream 1
    DMA_DeInit(DMA1_Stream1); 
    
    DMA_InitStructure.DMA_Channel = DMA_Channel_4;
	//外设地址
	DMA_InitStructure.DMA_PeripheralBaseAddr = (rt_uint32_t)(&MPU_USART->DR);
	//内存地址
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)rxbuf;
	//dma传输方向单向
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	//设置DMA在传输时缓冲区的长度
	DMA_InitStructure.DMA_BufferSize = MPU_BUF_LEN;
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
    
    
	DMA_Init(DMA1_Stream1,&DMA_InitStructure);
	//使能通道1
	DMA_Cmd(DMA1_Stream1,ENABLE);  
    // Enable DMA for MPU USART
    USART_DMACmd(MPU_USART, USART_DMAReq_Rx, ENABLE);
    
    /* NVIC Configuration */
    
    // Enable the USART3 Interrupt
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}