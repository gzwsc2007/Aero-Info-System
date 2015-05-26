/* 
 * NRF24L01.c
 * Ported from Mike McCauley's (mikem@open.com.au) NRF24 library for Arduino.
 *
 * Edited by Anson Wang
 *
 * This module provides interface for communicating with the NRF24L01 radio.
 *
 */

#include "stm32f4xx.h"
#include <rtthread.h>
#include "NRF24L01.h"
#include "finsh.h"

/* Public global variables */
struct NRF24_Buf_Struct NRF24_RX_Buf;
struct NRF24_Buf_Struct NRF24_TX_Buf;
struct rt_mutex NRF24_RX_Lock;  // mutex lock for the RX buffer
struct rt_mutex NRF24_TX_Lock;  // mutex lock for the TX buffer

/* Private global variables */
volatile static rt_uint8_t status;  // status of the NRF24L01 radio
struct rt_semaphore sem_sent;  // used to wait for transmission to complete
struct rt_semaphore sem_rx;

/* NRF24L01 util functions */
//void printRegisters(void); // moved to .h file
static rt_uint8_t statusRead(void);
static void flushTx(void);
static void flushRx(void);
static void setChannel(rt_uint8_t channel);
static void setThisAddress(rt_uint8_t *address, rt_uint8_t len);
static void setTransmitAddress(rt_uint8_t *address, rt_uint8_t len);
static void setPayloadSize(rt_uint8_t size);
static void setRF(rt_uint8_t data_rate, rt_uint8_t power);
static void powerDown(void);
static void powerUpRx(void);
static void powerUpTx(void);

/* Low-level SPI functions */
static rt_uint8_t SPI_RW(rt_uint8_t byte);
static rt_uint8_t SPI_ReadReg(rt_uint8_t reg);
static rt_uint8_t SPI_WriteReg(rt_uint8_t reg, rt_uint8_t byte);
static void SPI_BurstRead(rt_uint8_t command, rt_uint8_t* buf, rt_uint8_t len);
static void SPI_BurstWrite(rt_uint8_t command, rt_uint8_t* src, rt_uint8_t len);
static void SPI_BurstReadRegister(rt_uint8_t reg, rt_uint8_t *dest, 
                                  rt_uint8_t len);
static void SPI_BurstWriteRegister(rt_uint8_t reg, rt_uint8_t *src, 
                                   rt_uint8_t len);
static void Interface_Init(void);
static void delayus(unsigned long);

static volatile rt_bool_t txe = RT_TRUE;

/* TODO: This might not be a good idea for MAVlink implementation
 * The Radio_Thread, responsible for transmitting and receiving message at a constant
 * period (about 25 times per second).
 */
void radio_thread_entry(void *parameter)
{
    rt_uint32_t cnt = 0;
    rt_uint8_t ack_payload[] = {'d','e','a','d','\0'};
    rt_bool_t more = RT_FALSE;
    
    // initialize the "sem_sent" semaphore
    /*
    if (rt_sem_init(&sem_sent, "sent", 0, RT_IPC_FLAG_FIFO) != RT_EOK)
        rt_kprintf("rt_sem_init error\n");
    if (rt_mutex_init(&NRF24_RX_Lock, "RX_Lock", RT_IPC_FLAG_FIFO) != RT_EOK)
        rt_kprintf("rt_mutex_init error\n");
    if (rt_mutex_init(&NRF24_TX_Lock, "TX_Lock", RT_IPC_FLAG_FIFO) != RT_EOK)
        rt_kprintf("rt_mutex_init error\n");
    */
    powerUpRx();
    
    // main loop: propagate message to ground station about every 40 ms
    while(1)
    {
        /*
        rt_mutex_take(&NRF24_RX_Lock, RT_WAITING_FOREVER); 
        // check if there is any available message received
        if (NRF24_Recv(NRF24_RX_Buf.data, &(NRF24_RX_Buf.len)) == RT_TRUE)
        {
            // broadcast an event
        }
        rt_mutex_release(&NRF24_RX_Lock);
        
        // Send a message and suspend the thread until transmission completes.
        NRF24_Send(NRF24_TX_Buf.data, NRF24_TX_Buf.len, RT_FALSE);
        
        // Enter RX_Mode and wait for 40 ms
        rt_thread_delay(40);
        */
        NRF24_CE_HIGH();
        if (rt_sem_take(&sem_rx, 5000) != -RT_ETIMEOUT) {
          NRF24_CE_LOW();
          while (NRF24_Recv(NRF24_RX_Buf.data, &(NRF24_RX_Buf.len), &more) == RT_TRUE)
          {
              // broadcast an event
              rt_kprintf("%s\t%d\n",NRF24_RX_Buf.data, cnt++);
              
              // Put an ACK payload
   
              SPI_BurstWrite(NRF24_COMMAND_W_ACK_PAYLOAD, ack_payload, sizeof(ack_payload));
              
              if (!more) {
                  if (!(SPI_ReadReg(NRF24_REG_17_FIFO_STATUS) & NRF24_FIFO_RX_EMPTY)) {
                      printf("Shit!\r\n");
                  }
                  break;
              }
          }
        } else {
            printRegisters();
        }
    }
}

/*
 * Called by rt_application_init() in application.c
 */
void NRF24_Init(void)
{
    static rt_uint8_t other_addr[5] = {0xDE,0xAD,0xBE,0xEF,0x42};
    static rt_uint8_t this_addr[5] = {0x68,0x86,0x66,0x88,0x28};
    
    // initialize the "sem_sent" semaphore
    if (rt_sem_init(&sem_sent, "sent", 0, RT_IPC_FLAG_FIFO) != RT_EOK)
        rt_kprintf("rt_sem_init error\n");
    if (rt_sem_init(&sem_rx, "rx", 0, RT_IPC_FLAG_FIFO) != RT_EOK)
        rt_kprintf("rt_sem_init error\n");
    
    // initialize the peripherals needed for comm
    Interface_Init();

    // power down at first
    powerDown();

    // clear interrupts
    SPI_WriteReg(NRF24_REG_07_STATUS, NRF24_RX_DR | NRF24_TX_DS | NRF24_MAX_RT);
    setChannel(40);
//    setPayloadSize(32);
    setTransmitAddress(this_addr, 5);
    //setThisAddress(this_addr,5);
    setRF(NRF24DataRate250kbps, NRF24TransmitPower0dBm);
    
    SPI_WriteReg(NRF24_REG_1D_FEATURE, NRF24_EN_DPL | NRF24_EN_ACK_PAY); // Enable DPL and ACK payload
    SPI_WriteReg(NRF24_REG_1C_DYNPD, NRF24_DPL_P0);
    
    // flush FIFOs
    flushTx();
    flushRx();
    
    powerUpRx();
}


/*
 * Interrupt handler for the IRQ pin of the radio. Whenever RX_DR, TX_DS, or MAX_RT
 * is set, an interrupt would be triggered.
 */
void EXTI9_5_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line5) == SET)
    {     
        // Update the radio status
        status = statusRead(); 
        
        // if TX_DS or MAX_RT, relase sem_sent to notify radio_thread of transmission complete
        if (status & NRF24_MAX_RT)
        {
//            flushTx();
//            rt_sem_release(&sem_sent);
        }
        else if (status & NRF24_TX_DS) {
            txe = RT_TRUE;
 //           rt_sem_release(&sem_sent);  
        }
        
        if (status & NRF24_RX_DR) { // Oh man this should be a separate IF (not ELSE IF) !!!
            rt_sem_release(&sem_rx);
        }
        
        // Clear interrupts
        SPI_WriteReg(NRF24_REG_07_STATUS, NRF24_RX_DR | NRF24_TX_DS | NRF24_MAX_RT);
        
        // Clear IT pending bits
        EXTI_ClearITPendingBit(EXTI_Line5);
    }
}


/*
 * Blocking send one byte to the SPI slave, and return the received byte.
 * Note: need to manually pull low CS pin.
 */
static rt_uint8_t SPI_RW(rt_uint8_t byte)
{
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
    
    SPI_I2S_SendData(SPI1, byte);

    // wait if the SPI bus is busy
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
    
    // return the data received
    return SPI_I2S_ReceiveData(NRF24_SPI_PORT);
}


/*
 * Read the value contained in the selected register (not status!)
 */
static rt_uint8_t SPI_ReadReg(rt_uint8_t reg)
{
    rt_uint8_t val;
    NRF24_CE_LOW();
    NRF24_CSN_LOW();
    //delayus(20);
    
    reg = (reg & NRF24_REGISTER_MASK) | NRF24_COMMAND_R_REGISTER;
    SPI_RW(reg);                     // select the register
    val = SPI_RW(NRF24_COMMAND_NOP); // send dummy byte to read
    
    NRF24_CSN_HIGH();
    delayus(2);
    return val;
}


/*
 * Write one byte to a selected register and return the status byte.
 * To be used only with NRF24L01
 */
static rt_uint8_t SPI_WriteReg(rt_uint8_t reg, rt_uint8_t byte)
{
    rt_uint8_t status;
    NRF24_CE_LOW();
    NRF24_CSN_LOW();  // pull CS low
   // delayus(20);
    
    reg = (reg & NRF24_REGISTER_MASK) | NRF24_COMMAND_W_REGISTER;
    status = SPI_RW(reg); // select register and read its status byte
    SPI_RW(byte);         // send byte to the selected register

    NRF24_CSN_HIGH();  // pull CS high 
    delayus(2);
    return status;
}


/*
 * Read a sequence of len bytes from the slave, and put them in buf.
 */
static void SPI_BurstRead(rt_uint8_t command, rt_uint8_t* buf, rt_uint8_t len)
{
    NRF24_CE_LOW();
    NRF24_CSN_LOW();  // pull CS low
    //delayus(20);
    
    SPI_RW(command);
    while(len--)
    {
        *buf = SPI_RW(NRF24_COMMAND_NOP); // write dummy byte to read
        buf++;  // increase dest pointer
    }
    
    NRF24_CSN_HIGH();  // pull CS high 
    delayus(2);
}


/*
 * Write a sequence of len bytes to the slave.
 *
 * Code orginated from Mike McCauley (mikem@open.com.au)
 * Drawn from his NRF24 library for Arduino
 */
static void SPI_BurstWrite(rt_uint8_t command, rt_uint8_t* src, rt_uint8_t len)
{
    NRF24_CE_LOW();
    NRF24_CSN_LOW();  // pull CS low
   // delayus(20);
    
    SPI_RW(command);
    // perform a sequence of len writes
    while(len--) 
    {
        SPI_RW(*src); 
        src++;   // increase source pointer
    }
    
    NRF24_CSN_HIGH();  // pull CS high
    delayus(1);
}


/*
 * Read multiple bytes from an NRF24L01 register
 */
static void SPI_BurstReadRegister(rt_uint8_t reg, rt_uint8_t *dest, 
                                  rt_uint8_t len)
{
    SPI_BurstRead((reg & NRF24_REGISTER_MASK) | NRF24_COMMAND_R_REGISTER,
                  dest, len);
}

/*
 * Write multiple bytes to an NRF24L01 register
 */
static void SPI_BurstWriteRegister(rt_uint8_t reg, rt_uint8_t *src, 
                                   rt_uint8_t len)
{
    SPI_BurstWrite((reg & NRF24_REGISTER_MASK) | NRF24_COMMAND_W_REGISTER, 
                   src, len);
}


/*************************************************************************/
/**************** Some NRF24L01 Util functions ***************************/
/*************************************************************************/

/* Print out all the register values. Useful for debugging */
void printRegisters(void)
{
    rt_uint8_t registers[] = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                               0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
                               0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
                               0x1C, 0x1D};
    rt_uint8_t i;
    rt_uint8_t stat;
                               
    for (i = 0; i < sizeof(registers); i++)
    {
        stat = SPI_ReadReg(registers[i]);
        rt_kprintf("Reg %x: %x\n", registers[i], stat);
    }
}
FINSH_FUNCTION_EXPORT(printRegisters, "NRF24 REG");

/* 
 * Read the status of the radio. 
 * More specifically, RX_DR, TX_DS, MAX_RT, RX_P_NO, and TX_FULL.
 */
static rt_uint8_t statusRead(void)
{
    return SPI_ReadReg(NRF24_REG_07_STATUS);
}


static void flushTx(void)
{
    NRF24_CSN_LOW();  // pull CS low
    delayus(20);
    SPI_RW(NRF24_COMMAND_FLUSH_TX);
    NRF24_CSN_HIGH();  // pull CS high 
}


static void flushRx(void)
{
    NRF24_CSN_LOW();  // pull CS low
    delayus(20);
    SPI_RW(NRF24_COMMAND_FLUSH_RX);
    NRF24_CSN_HIGH();  // pull CS high 
}


/* RF frequency == (2400 + channel) Mhz. channel 0 ~ 50 */
static void setChannel(rt_uint8_t channel)
{
    SPI_WriteReg(NRF24_REG_05_RF_CH, channel & NRF24_RF_CH);
}


static void setThisAddress(rt_uint8_t *address, rt_uint8_t len)
{
    // Set RX_ADDR_P1 for this address
    SPI_BurstWriteRegister(NRF24_REG_0B_RX_ADDR_P1, address, len);
    // RX_ADDR_P2 is set to RX_ADDR_P1 with the LSByte set to 0xff, 
    // for use as a broadcast address
}


static void setTransmitAddress(rt_uint8_t *address, rt_uint8_t len)
{
    // Set both TX_ADDR and RX_ADDR_P0 for auto-ack with Enhanced shockwave
    SPI_BurstWriteRegister(NRF24_REG_0A_RX_ADDR_P0, address, len);
    SPI_BurstWriteRegister(NRF24_REG_10_TX_ADDR, address, len);
}


/* Payload size is configurable between 0 - 32 bytes */
static void setPayloadSize(rt_uint8_t size)
{
    SPI_WriteReg(NRF24_REG_11_RX_PW_P0, size);
    SPI_WriteReg(NRF24_REG_12_RX_PW_P1, size);
}


/* 
 * Set the data rate and transmission power. Only accept parameters defined
 * in the enum in NRF24L01.h
 */
static void setRF(rt_uint8_t data_rate, rt_uint8_t power)
{
    uint8_t value = (power << 1) & NRF24_PWR;
    // Ugly mapping of data rates to noncontiguous 2 bits:
    if (data_rate == NRF24DataRate250kbps)
	value |= (NRF24_RF_DR_LOW);
    else if (data_rate == NRF24DataRate2Mbps)
	value |= NRF24_RF_DR_HIGH;
    // else NRF24DataRate1Mbps, 00
    SPI_WriteReg(NRF24_REG_06_RF_SETUP, value);
}


static void powerDown(void)
{
    SPI_WriteReg(NRF24_REG_00_CONFIG, NRF24_DEFAULT_CONFIGURATION);
    NRF24_CE_LOW();
}


/* Configure the radio to RX mode with DEFAULT_CONFIGURATION */
static void powerUpRx(void)
{
    SPI_WriteReg(NRF24_REG_00_CONFIG, NRF24_DEFAULT_CONFIGURATION | 
                 NRF24_PWR_UP | NRF24_PRIM_RX);
    NRF24_CE_HIGH();
}


/* Configure the radio to TX mode with DEFAULT_CONFIGURATION */
static void powerUpTx(void)
{
    // Its the pulse high that puts us into TX mode
    NRF24_CE_LOW();
    SPI_WriteReg(NRF24_REG_00_CONFIG, NRF24_DEFAULT_CONFIGURATION | 
                 NRF24_PWR_UP);
    NRF24_CE_HIGH();
}

/* 
 * Transmit a data payload that is len bytes long. If noack is TRUE, then AUTO_ACK
 * is disabled and there will be no ACK.
 * 
 * Note: This functin puts the radio into TX_MODE. The Radio will return to Standby II 
 * mode after transmission is complete (since CE is kept high). Client may want to
 * manually switch the radio back to RX_MODE.
 *
 * The function will suspend the calling thread until the transmission is 
 * complete. Return value is the transmission status.
 */
rt_uint8_t NRF24_Send(rt_uint8_t *data, rt_uint8_t len, rt_bool_t noack)
{
    // clear interrupt bit if necessary
    if (statusRead() & NRF24_MAX_RT)
        SPI_WriteReg(NRF24_REG_07_STATUS, NRF24_MAX_RT);
    rt_thread_delay(1);
    sem_sent.value = 0; // reset semaphore value

    // write the payload
    if (noack)
        SPI_BurstWrite(NRF24_COMMAND_W_TX_PAYLOAD_NOACK, data, len);
    else
        SPI_BurstWrite(NRF24_COMMAND_W_TX_PAYLOAD, data, len);
    
    // wait until sending complete
    powerUpTx();
    rt_sem_take(&sem_sent, 500);
    
    return statusRead();
}


/* 
 * Check if there is avaialble payload in the RX_FIFO of the readio
 */
static rt_bool_t available(void)
{
    if (SPI_ReadReg(NRF24_REG_17_FIFO_STATUS) & NRF24_FIFO_RX_EMPTY)
        return RT_FALSE;
    // Manual says that messages > 32 octets should be discarded
    SPI_RW(NRF24_COMMAND_R_RX_PL_WID); // TODO: this is BUGGY!!
    if (SPI_RW(NRF24_COMMAND_NOP) > 32)
    {
        flushRx();
        return RT_FALSE;
    }
    
    return RT_TRUE;
}

rt_uint8_t NRF24_get_payload_len(void)
{
    rt_uint8_t len = 0;
    SPI_BurstRead(NRF24_COMMAND_R_RX_PL_WID, &len, sizeof(len));
    return len;
}

/*
 * Check if there is a valid payload in the RX_FIFO of the radio. If yes, copy it
 * to buf and set *len to the size of the copied payload (must be <= 32 bytes, so
 * buf should be a buffer with 32 bytes). If not, simply return false.
 *
 * Note: The radio should be configured into RX mode before calling this function.
 *
 * Return RT_TRUE if a payload is copied into buf, and RT_FALSE otherwise.
 */
rt_bool_t NRF24_Recv(rt_uint8_t *buf, rt_uint8_t *len, rt_bool_t *more)
{
    if (!*more && !(status & NRF24_RX_DR))
        return RT_FALSE;
    
    *len = NRF24_get_payload_len();
    if (*len <= 32) {
      SPI_BurstRead(NRF24_COMMAND_R_RX_PAYLOAD, buf, *len);
    } else {
        return RT_FALSE;
    }
    
    rt_thread_delay(1);
    
    // Check if there is more data to read
    rt_uint8_t fifo_stat = SPI_ReadReg(NRF24_REG_17_FIFO_STATUS);
    if (fifo_stat & NRF24_FIFO_RX_EMPTY) {
      *more = RT_FALSE;
    } else {
        *more = RT_TRUE;
    }
    
    return RT_TRUE;
}

/******************************************************************************/


/* 
 * Configure SPI1 Settings.
 */
static void SPI1_Configuration(void)
{
	SPI_InitTypeDef SPI_InitStruct;
    
    SPI_StructInit(&SPI_InitStruct);
    
    // set to full duplex mode, seperate MOSI and MISO lines
    SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex; 
	SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
    // CPOL = 0 , CPHA = 1 for NRF24L01
	SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;   
	SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge; 
	SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;
    // 84000kHz/256=328kHz < 400kHz
	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256; 
    // data is transmitted MSB first
	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
	 SPI_Init(SPI1, &SPI_InitStruct);
     
    // Enable SPI1.NSS as a GPIO
    SPI_SSOutputCmd(SPI1, ENABLE);
    // set the NSS management to internal and pull internal NSS high
    SPI_NSSInternalSoftwareConfig(SPI1, SPI_NSSInternalSoft_Set);
    
    SPI_CalculateCRC(SPI1, DISABLE);
	SPI_Cmd(SPI1, ENABLE); // enable SPI1
}


/* 
 * NOTE: This function need to be changed if NRF24_SPI_PORT changes
 *
 * Configure pins used by SPI1
 *
 * PA5 = SCK
 * PA6 = MISO
 * PA7 = MOSI
 * CS pin is defined in NRF24L01.h
 */
static void GPIO_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    
    // configure pins used by SPI1
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_6 | GPIO_Pin_5;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	// connect SPI1 pins to SPI alternate function
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);
	
	// Configure the CS pin (NRF24_CS_PIN)
	GPIO_InitStruct.GPIO_Pin = NRF24_CS_PIN;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(NRF24_CS_PORT, &GPIO_InitStruct);
	
    // de-select slave initially
	NRF24_CSN_HIGH();
    
    // initialize the CE (chip enable) pin
    GPIO_InitStruct.GPIO_Pin = NRF24_CE_PIN;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(NRF24_CE_PORT, &GPIO_InitStruct);
    
    // default CE LOW
    NRF24_CE_LOW();
    
    // configure the IRQ pin to input mode
    GPIO_InitStruct.GPIO_Pin = NRF24_IRQ_PIN;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
    GPIO_Init(NRF24_IRQ_PORT, &GPIO_InitStruct);
}


static void EXTI_Configuration(void)
{
    EXTI_InitTypeDef EXTI_InitStruct;
    
    // connect the IRQ pin to its EXTI line (Note: hard-coded port and pin value!!!)
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource5);
    
    EXTI_InitStruct.EXTI_Line = EXTI_Line5;  // IRQ is pin 5
    EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;  
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;
    
    EXTI_Init(&EXTI_InitStruct);
}


static void NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStruct;
    
    NVIC_InitStruct.NVIC_IRQChannel = EXTI9_5_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    
    NVIC_Init(&NVIC_InitStruct);
}


/*
 * Note: This function needs to be changed if NRF24_SPI_PORT changes
 *
 * Enable peripheral clocks
 */
static void RCC_Configuration(void)
{
	// enable GPIOA clock
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    // enable GPIOC clock
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    // configure APB2 prescaler
   // RCC_PCLK2Config(RCC_HCLK_Div16);
    // enable SPI clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
    // enable SYSCFG clock so as to write to SYSCFG
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
}


/*
 * Initialize the peripherals used to communicate with NRF24L01
 */
static void Interface_Init(void)
{
    RCC_Configuration();
    GPIO_Configuration();
    SPI1_Configuration();
    EXTI_Configuration();
    NVIC_Configuration();
}

static void delayus(unsigned long us) {
    unsigned long i;
    while(us--) {
        i = 100;
        while(i--);
    }
}