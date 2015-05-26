/* 
 * I2CSensors.c
 *
 * Author: Anson Wang
 *
 * This module provides interface for accessing data from some sensors that run 
 * on I2C protocol. Currently supports the BMP085 barometer, the HMC5883L
 * digital compass and the ADS1115 analog-digital converter.
 * 
 * Note: The I2C driver functions are written by Elia. 
 * Source: eliaelectronics.com/stm32f4-tutorials/stm32f4-i2c-master-tutorial
 */

#include "stm32f4xx.h"
#include <rtthread.h>
#include "I2CSensors.h"
#include "rwlock.h"
#include <math.h>

/* Private Helper Function Declarations */

// ADS1115 ADC util functions
static void ADS1115_Init(void);

// BMP085 altimeter util functions
static rt_uint16_t BMP085_readRawTemp(void);
static rt_uint32_t BMP085_readRawPres(void);
static void BMP085_Init(void);

// HMC5883 compass util functions
static rt_bool_t HMC5883_testCommunication(void);
static rt_bool_t HMC5883_Init(void);
static void HMC5883_updateXZY(void);

// Read/Write one/two bytes from a specific device. Note that the dev_addr is 
// the shifted 7-bit address, with its lowest bit (bit0) being 0. "reg" is the 
// register to read from. All operations are MSB first.
static rt_uint16_t I2C_readTwoBytes(rt_uint8_t dev_addr, rt_uint8_t reg);
static rt_uint8_t I2C_readOneByte(rt_uint8_t dev_addr, rt_uint8_t reg);
static void I2C_writeTwoBytes(rt_uint8_t dev_addr, rt_uint8_t reg, rt_uint16_t word);
static void I2C_writeOneByte(rt_uint8_t dev_addr, rt_uint8_t reg, rt_uint8_t byte);
static void RCC_Configuration(void);
static void GPIO_Configuration(void);
static void I2C1_Configuration(void);
static void EXTI_Configuration(void);
static void NVIC_Configuration(void);

// Low-level I2C driver functions (by Elia)
static void I2C_start(I2C_TypeDef* I2Cx, rt_uint8_t address, rt_uint8_t direction);
static void I2C_write(I2C_TypeDef* I2Cx, rt_uint8_t data);
static rt_uint8_t I2C_read_ack(I2C_TypeDef* I2Cx);
static rt_uint8_t I2C_read_nack(I2C_TypeDef* I2Cx);
static void I2C_stop(I2C_TypeDef* I2Cx);

/* End Private Helper Function Declarations */


/* Private Global Variables */

// ADS1115 ADC
static adsGain_t PGA_gain = GAIN_ONE; // default gain
static double fullscale = 4.096; // default gain
struct rt_semaphore sem_adc;  // used to wait for conversion to complete
static double Vdd;
static double Vbatt;

// current sensing
static double Vcurr;

// airspeed readings
static double Vairspeed;
static double Vaspd_neutral;
static double Vaspd_factor;

// BMP085 altimeter
static rt_int16_t ac1, ac2, ac3, b1, b2, mb, mc, md;  // calibration values
static rt_uint16_t ac4, ac5, ac6;
static rt_int32_t b5; // b5 is calculated in bmp085GetTemperature(...), this variable 
                      // is also used in bmp085GetPressure(...) so ...Temperature(...) 
                      // must be called before ...Pressure(...).
static rt_uint16_t temperature;  // converted readings
static rt_uint32_t pressure;
static rt_uint32_t press_ref; // pressure at ground level
static rwlock_t baroLock;

// HMC5883 Compass
static rt_int16_t magX, magY, magZ; // raw data measured by HMC5883

/* End Private Global Variables */


/*
 * Responsible for converting voltage readings from multiple analog sensors,
 * including the airspeed sensor, the current sensor and the battery voltage
 * sensor.
 *
 * Refreshing rate for each sensor:
 *  - Airspeed: 30 Hz
 *  - Current: 30 Hz 
 * - Batt Volt: 5 - 10 Hz
 */
void ADS1115_thread_entry(void *param)
{
    rt_uint16_t cnt = 0;
    rt_uint8_t subCnt = 0;
    double prevAspd = 0;
    
    ADS1115_Init();
    
    //if (rt_sem_init(&sem_adc, "sem_adc", 0, RT_IPC_FLAG_FIFO) != RT_EOK)
    //    rt_kprintf("rt_sem_init error\n");
    
    ADS1115_setGain(GAIN_TWOTHIRDS);
    
    // measure the 5V supply voltage so that I can determine the midpoint of 
    // many analog sensors.
    Vdd = ((double)ADS1115_readSingleEnded(ADC_CHAN_VDD)) * fullscale /
           (double)(0x7FFF);
    
    rt_thread_delay(10000);
    
    // find neutral positions for airspeed
    calibrateAirspeed();
    prevAspd = Vaspd_neutral;
    
    while(1)
    {
        // Vcurr and Vairspeed updated at about 30 Hz
        Vcurr = ((double)ADS1115_readSingleEnded(ADC_CHAN_VCURRENT)) * 
                fullscale / (double)(0x7FFF);
        
        Vairspeed = ((double)ADS1115_readSingleEnded(ADC_CHAN_VAIRSPEED)) * 
                    fullscale / (double)(0x7FFF);
        
        Vairspeed = prevAspd * 0.85 + (Vairspeed * 0.15);
        prevAspd = Vairspeed;
        //rt_kprintf("%d km/h\n", (rt_int32_t)(getAirspeed() * 3.6));
        //rt_kprintf("%d pa\n", (rt_int32_t)((Vairspeed - Vaspd_neutral)*1000.0/(0.2*Vdd)));
        
        // Vbatt updated at about 5 Hz
        if (cnt >= 6) {
            cnt = 0;
            
            Vbatt = ((double)ADS1115_readSingleEnded(ADC_CHAN_VBATT)) * 
                    fullscale / (double)(0x7FFF);
            
          //  rt_kprintf(" Vcurr = %d\n", (rt_int32_t)(Vcurr*1000.0));
          //  rt_kprintf("       Vdd = %d\n", (rt_int32_t)(Vdd*1000.0));
          //  rt_kprintf("                 Vbatt = %d\n", (rt_int32_t)(Vbatt*1000.0));
            
            
            subCnt++;
            
            if (subCnt >= 10) {
                subCnt = 0;
                
                Vdd = ((double)ADS1115_readSingleEnded(ADC_CHAN_VDD)) * fullscale /
               (double)(0x7FFF);
               
                Vaspd_factor = 2000.0 / (0.2 * Vdd) / 1.15; // 1.15 kg/m^3 at 30 celcius
            }
            // given the extra time converting Vbatt, just skip the delay.
            continue;
        }              
        
        cnt++;
        rt_thread_delay(30);
    }
}

// return battery voltage in V
double getBattVoltage(void)
{
    return Vbatt * VBATT_SCALING_FACTOR;
}

// return battery current in Amps
double getBattCurrent(void)
{
    double deltaV = Vcurr - 2.5;
    
    if (deltaV > 0) return deltaV / 0.04; // sensitivity is 40mV / A
    else return 0.0;
}

void calibrateAirspeed(void)
{
    double v = ((double)ADS1115_readSingleEnded(ADC_CHAN_VAIRSPEED)) * 
             fullscale / (double)(0x7FFF);
    double tempV;
    
    // take the average reading
    for (rt_uint8_t i = 0; i < 200; i++) {
        tempV = ((double)ADS1115_readSingleEnded(ADC_CHAN_VAIRSPEED)) * 
                 fullscale / (double)(0x7FFF);
        v = 0.1 * tempV + 0.9 * v;
    }
    
    Vaspd_neutral = v - 0.005;
    Vaspd_factor = 2000.0 / (0.2 * Vdd) / 1.15; // 1.15 kg/m^3 at 30 celcius
    
}

// return airspeed in m/s
double getAirspeed(void)
{    
    if (Vairspeed > Vaspd_neutral) 
        return sqrt((Vairspeed - Vaspd_neutral)*Vaspd_factor);
    else 
        return -sqrt((Vaspd_neutral - Vairspeed)*Vaspd_factor);
    
}

/*
 * Responsible for gathering data from the BMP085 barometer constantly, and 
 * convert the readings into temperature (0.01 C), pressure (Pa) and relative 
 * altitude (m)
 *
* Refreshing rate:
*  - Temperature: 1 - 2 Hz
*  - Pressure (Altitude): 20 Hz
 */
void BMP085_thread_entry(void *parameters)
{
    rt_uint8_t cnt = 20;
    rwlock_init(&baroLock);
    
    // initialize the ground level pressure
    temperature = BMP085_getTemperature();
    press_ref = BMP085_getPressure();
    press_ref += BMP085_getPressure(); 
    press_ref += BMP085_getPressure();
    press_ref += BMP085_getPressure(); // measure several times
    press_ref = press_ref / 4; // and take the average
    
    rt_kprintf("BMP085 pressure ref: %d\n", press_ref);
    
    // the loop is running at about 18 Hz
    while (1)
    {
        // update temperature reading rate about 5 Hz
        if (cnt >= 5)
        {
            cnt = 0;
            //rwlock_wrlock(&baroLock);
            temperature = BMP085_getTemperature();
            //rwlock_wrunlock(&baroLock);
        }
        
        // take the pressure reading
       // rwlock_wrlock(&baroLock);
        pressure = BMP085_getPressure();
       // rwlock_wrunlock(&baroLock);
        
        // might want to calculate altitude here
        
        cnt++;
        rt_thread_delay(25); 
        
        // ignore this line for now
       // HMC5883_updateXZY();
    }
}


void I2CSensors_Init(void)
{
    RCC_Configuration();
    GPIO_Configuration();
    I2C1_Configuration();
    EXTI_Configuration();
    NVIC_Configuration();
    
    BMP085_Init();
    //if (!HMC5883_Init())
    //    rt_kprintf("HMC5883L Init failed\n");
}

/*********** ADS1115 ADC **********************/

/* Default gain set to one. Disable Comparator. Use RDY pin for conversion ready */
static void ADS1115_Init()
{
    rt_uint16_t hi_thresh_val = 0x8000;
    rt_uint16_t lo_thresh_val = 0x0000;
    //rt_uint16_t check = 0;
    //rt_int16_t shit = 0;
    
    ADS1115_setGain(GAIN_TWOTHIRDS); // need to first measure Vdd
    
    I2C_writeTwoBytes(ADS1115_ADDR, ADS1115_REG_POINTER_LOWTHRESH, lo_thresh_val);
    I2C_writeTwoBytes(ADS1115_ADDR, ADS1115_REG_POINTER_HITHRESH, hi_thresh_val);

    //check = (rt_int16_t)I2C_readTwoBytes(ADS1115_ADDR, ADS1115_REG_POINTER_HITHRESH);
    //shit = check;
    
    return;
}

/*
 * Read the converted voltage from a designated channel in the ADS1115.
 */
rt_int16_t ADS1115_readSingleEnded(rt_uint8_t channel) {
    rt_int16_t config;
    
    if (channel > 3)
        return 0;
    
    // set up default config values
    config = ADS1115_REG_CONFIG_CQUE_1CONV    | // Disable the comparator (default val)
             ADS1115_REG_CONFIG_CLAT_NONLAT  | // Non-latching (default val)
             ADS1115_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
             ADS1115_REG_CONFIG_CMODE_TRAD   | // Traditional comparator (default val)
             ADS1115_REG_CONFIG_DR_860SPS    | // 860 samples per second (default)
             ADS1115_REG_CONFIG_MODE_SINGLE;   // Single-shot mode (default)
    
    // set PGA gain
    config |= PGA_gain;
    
    // Set single-ended input channel
    switch (channel) {
        case 0:
            config |= ADS1115_REG_CONFIG_MUX_SINGLE_0;
            break;
        case 1:
            config |= ADS1115_REG_CONFIG_MUX_SINGLE_1;
            break;
        case 2:
            config |= ADS1115_REG_CONFIG_MUX_SINGLE_2;
            break;
        case 3:
            config |= ADS1115_REG_CONFIG_MUX_SINGLE_3;
            break;
    }
    
    // Set the "start single-conversion" bit
    config |= ADS1115_REG_CONFIG_OS_SINGLE;
    
    //sem_adc.value = 0;
    // Apply the config and start the conversion immediately
    I2C_writeTwoBytes(ADS1115_ADDR, ADS1115_REG_POINTER_CONFIG, config);
    
    // suspend the calling thread until conversion is ready. sem_adc will be
    // released as soon as the RDY pin is pulled low and an interrupt is 
    // triggered.
    //rt_sem_take(&sem_adc, 5);

    rt_thread_delay(5);
    
    return (rt_int16_t)I2C_readTwoBytes(ADS1115_ADDR, ADS1115_REG_POINTER_CONVERT);
}

/*
 * Interrupt handler for the RDY pin of ADS1115. Active low when conversion
 * completes.
 */
void EXTI0_IRQHandler(void) // TODO: Test this!!!!
{
    if (EXTI_GetITStatus(EXTI_Line0) == SET)
    {
        // Clear IT pending bits
        EXTI_ClearITPendingBit(EXTI_Line0);
        
        // release the blocking semaphore so that the ADS1115 thread can
        // resume.
        rt_sem_release(&sem_adc);
    }
}

/*
 * Set the PGA gain of the ADS1115. The input parameter must be of the
 * adsGain_t enum type
 */
void ADS1115_setGain(adsGain_t gain) {
    PGA_gain = gain;
    
    // adjust the full scale range accordingly
    switch(PGA_gain) {
        case GAIN_TWOTHIRDS:
            fullscale = 6.144;
            break;
        case GAIN_ONE:
            fullscale = 4.096;
            break;
        case GAIN_TWO:
            fullscale = 2.048;
            break;
        case GAIN_FOUR:
            fullscale = 1.024;
            break;
        case GAIN_EIGHT:
            fullscale = 0.512;
            break;
        case GAIN_SIXTEEN:
            fullscale = 0.256;
            break;
    }
}

/*
 * Return the current PGA gain setting.
 */
adsGain_t ADS1115_getGain(void) {
    return PGA_gain;
}
    
/*********** End ADS1115 **********************/


/************ BMP085 Barometer ****************/

// calculate the relative altitude (wrt to ground level)
float BMP085_presToAlt(rt_int32_t pres)
{
    float altitude;
    float x;
    x = pow((float)pres / (float)press_ref , 1.0/5.255);
    altitude = 44330.0 * (1.0 - x);
    return altitude;
}

void BMP085_Init(void)
{
    ac1 = I2C_readTwoBytes(BMP085_ADDR, 0xAA);
    ac2 = I2C_readTwoBytes(BMP085_ADDR, 0xAC);
    ac3 = I2C_readTwoBytes(BMP085_ADDR, 0xAE);
    ac4 = I2C_readTwoBytes(BMP085_ADDR, 0xB0);
    ac5 = I2C_readTwoBytes(BMP085_ADDR, 0xB2);
    ac6 = I2C_readTwoBytes(BMP085_ADDR, 0xB4);
    b1 = I2C_readTwoBytes(BMP085_ADDR, 0xB6);
    b2 = I2C_readTwoBytes(BMP085_ADDR, 0xB8);
    mb = I2C_readTwoBytes(BMP085_ADDR, 0xBA);
    mc = I2C_readTwoBytes(BMP085_ADDR, 0xBC);
    md = I2C_readTwoBytes(BMP085_ADDR, 0xBE);
}


/*
 * Return the measured temperature and pressure into the pointers provided.
 * The unit of temp is in 0.1 Celcius and the unit for pres is in Pa.
 * 
 * This function is non-blocking. The data gathering process is handled in the background.
 */
void BMP085_getTempAndPres(rt_int16_t *temp, rt_int32_t *pres)
{
    //rwlock_rdlock(&baroLock);
    if(temp != RT_NULL) *temp = temperature;
    if(pres != RT_NULL) *pres = pressure;
   // rwlock_rdunlock(&baroLock);
}


/*
 * Read the raw pressure data and convert it to real pressure in Pa
 *
 * Note: the variable "b5" is required in this function. So BMP085_getTemperature(..) must
 * be called prior to this function.
 *
 * Source: https://www.sparkfun.com/tutorials/253
 */
rt_int32_t BMP085_getPressure(void)
{
    rt_uint32_t up = BMP085_readRawPres();
    rt_int32_t x1, x2, x3, b3, b6, p;
    rt_uint32_t b4, b7;

    b6 = b5 - 4000;
    // Calculate B3
    x1 = (b2 * (b6 * b6)>>12)>>11;
    x2 = (ac2 * b6)>>11;
    x3 = x1 + x2;
    b3 = (((((rt_int32_t)ac1)*4 + x3)<<BMP085_OSS) + 2)>>2;

    // Calculate B4
    x1 = (ac3 * b6)>>13;
    x2 = (b1 * ((b6 * b6)>>12))>>16;
    x3 = ((x1 + x2) + 2)>>2;
    b4 = (ac4 * (rt_uint32_t)(x3 + 32768))>>15;

    b7 = ((rt_uint32_t)(up - b3) * (50000>>BMP085_OSS));
    if (b7 < 0x80000000)
        p = (b7<<1)/b4;
    else
        p = (b7/b4)<<1;

    x1 = (p>>8) * (p>>8);
    x1 = (x1 * 3038)>>16;
    x2 = (-7357 * p)>>16;
    p += (x1 + x2 + 3791)>>4;

    return p;
    
}


/*
 * Read the raw temperature data from the sensor and then convert it to real temperature
 * in 0.1 Celcius
 *
 * Source: https://www.sparkfun.com/tutorials/253
 */
rt_int16_t BMP085_getTemperature(void)
{
    rt_uint16_t ut = BMP085_readRawTemp();
    
    rt_int32_t x1, x2;
    
    x1 = (((rt_int32_t)ut - (rt_int32_t)ac6) * (rt_int32_t)ac5) >> 15;
    x2 = ((rt_int32_t)mc << 11) / (x1 + md);
    b5 = x1 + x2;
    
    return ((b5 + 8) >> 4);
}


/*
 * Start the temperature conversion and wait until it completes. Then, return the raw data.
 * According to the datasheet, the max waiting time for temp conversion is 4.5 ms. So I will
 * suspend the calling thread for 5 ms to wait for completion.
 *
 * Note, temperature measurement only needs to be taken about every 1 second.
 */
static rt_uint16_t BMP085_readRawTemp(void)
{
    // issue the start-conversion command
    I2C_writeOneByte(BMP085_ADDR, BMP085_CTRL_REG, BMP085_READ_UT);
    
    // wait for conversion to complete (5 ms)
    rt_thread_delay(5);
    
    return (rt_uint16_t)I2C_readTwoBytes(BMP085_ADDR, BMP085_MSB_REG);
}


/*
 * Start the pressure conversion and wait until it completes. According to the data sheet,
 * the waiting time depends on the OVERSAMPLING setting:
 *     osrs == 0 (Low Power Mode)        ---->  4.5  ms
 *     osrs == 1 (Standard Mode)         ---->  7.5  ms
 *     osrs == 2 (High Resolution)       ---->  13.5 ms
 *     osrs == 3 (Ultra-high Resolution) ---->  25.5 ms
 */
static rt_uint32_t BMP085_readRawPres(void)
{
    rt_uint8_t msb, lsb, xlsb;
    rt_uint32_t up = 0;
    
    // issue the start-conversion command
    I2C_writeOneByte(BMP085_ADDR, BMP085_CTRL_REG, 
                     BMP085_READ_UP + (BMP085_OSS << 6));
    
    // wait for conversion to complete
    rt_thread_delay(2 + (3 << BMP085_OSS));
    
    // Read registers MSB, LSB and XLSB
    I2C_start(I2C1, BMP085_ADDR, I2C_Direction_Transmitter);
    I2C_write(I2C1, BMP085_MSB_REG);  // select MSB register to read from
    I2C_stop(I2C1);
    
    I2C_start(I2C1, BMP085_ADDR, I2C_Direction_Receiver);
    msb = I2C_read_ack(I2C1);
    lsb = I2C_read_ack(I2C1);
    xlsb = I2C_read_nack(I2C1);
    
    up = (((rt_uint32_t) msb << 16) | ((rt_uint32_t) lsb << 8) | 
          (rt_uint32_t) xlsb) >> (8 - BMP085_OSS);
    
    return up;
}

/******************* End BMP085 Barometer **************************/


/******************* HMC5883L Digital Compass **********************/

static rt_bool_t HMC5883_Init(void)
{
    // step 1: verify that I2C communication is good
    if(!HMC5883_testCommunication()) return RT_FALSE;
    // step 2: set configuration register A
    I2C_writeOneByte(HMC5883_ADDR,HMC5883_CRA_REG,0x70); // clear CRA7; others default.
    // step 3: set mode register
    I2C_writeOneByte(HMC5883_ADDR,HMC5883_MR_REG,0x00); // continuous measurement mode

    return RT_TRUE; 
}


/*
 * Read the identification registers in the HMC5883L to make sure
 * I2C communication is OK.
 */
static rt_bool_t HMC5883_testCommunication(void)
{
    rt_uint8_t irA, irB, irC;
    irA = I2C_readOneByte(HMC5883_ADDR, HMC5883_IRA_REG);
    irB = I2C_readOneByte(HMC5883_ADDR, HMC5883_IRB_REG);
    irC = I2C_readOneByte(HMC5883_ADDR, HMC5883_IRC_REG);
    if (irA == 72 && irB == 52 && irC == 51) 
        return RT_TRUE;
    else 
        return RT_FALSE;
}


// Read out the raw measurement of the X,Z,Y axes.
static void HMC5883_updateXZY(void)
{
    I2C_start(I2C1, HMC5883_ADDR, I2C_Direction_Transmitter);
    I2C_write(I2C1, 0x03); // select the X_MSB register to read
    I2C_stop(I2C1);
    
    I2C_start(I2C1, HMC5883_ADDR, I2C_Direction_Receiver);
    
    magX = I2C_read_ack(I2C1);
    magX <<= 8;
    magX |= I2C_read_ack(I2C1);
    
    magZ = I2C_read_ack(I2C1);
    magZ <<= 8;
    magZ |= I2C_read_ack(I2C1);
    
    magY = I2C_read_ack(I2C1);
    magY <<= 8;
    magY |= I2C_read_nack(I2C1);
}

/******************* End HMC5883L Digital Compass ******************/


// Read two bytes from a specific device. Note that the dev_addr is the shifted 7-bit 
// address, with its lowest bit (bit0) being 0. "reg" is the register to read from.
static rt_uint16_t I2C_readTwoBytes(rt_uint8_t dev_addr, rt_uint8_t reg)
{
    rt_uint16_t temp;
    
    // start a transmission in Master Transmitter mode
    I2C_start(I2C1, dev_addr, I2C_Direction_Transmitter);
    // select the register to read from
    I2C_write(I2C1, reg);
    I2C_stop(I2C1);
    
    // restart a transmission in Master Receiver mode
    I2C_start(I2C1, dev_addr, I2C_Direction_Receiver);
    // read two bytes from the slave (MSB first)
    temp = I2C_read_ack(I2C1);
    temp = temp << 8;
    temp |= I2C_read_nack(I2C1);
    
    return temp;
}


// Read one bytes from a specific device. Note that the dev_addr is the shifted 7-bit 
// address, with its lowest bit (bit0) being 0. "reg" is the register to read from.
static rt_uint8_t I2C_readOneByte(rt_uint8_t dev_addr, rt_uint8_t reg)
{
    // start a transmission in Master Transmitter mode
    I2C_start(I2C1, dev_addr, I2C_Direction_Transmitter);
    // select the register to read from
    I2C_write(I2C1, reg);
    I2C_stop(I2C1);
    
    // restart a transmission in Master Receiver mode
    I2C_start(I2C1, dev_addr, I2C_Direction_Receiver);
    // read one byte from the slave
    return I2C_read_nack(I2C1);
}


// Write two bytes to a specific register (MSB first)
static void I2C_writeTwoBytes(rt_uint8_t dev_addr, rt_uint8_t reg, rt_uint16_t word)
{
    I2C_start(I2C1, dev_addr, I2C_Direction_Transmitter);
    I2C_write(I2C1, reg); // select the register to write
    I2C_write(I2C1, (rt_uint8_t)(word >> 8));
    I2C_write(I2C1, (rt_uint8_t)(word & 0xFF));
    I2C_stop(I2C1);
}


// Write one byte to a specific register
static void I2C_writeOneByte(rt_uint8_t dev_addr, rt_uint8_t reg, rt_uint8_t byte)
{
    I2C_start(I2C1, dev_addr, I2C_Direction_Transmitter);
    I2C_write(I2C1, reg); // select the register to write
    I2C_write(I2C1, byte);
    I2C_stop(I2C1);
}


/* This function issues a start condition and 
 * transmits the slave address + R/W bit
 * 
 * Parameters:
 * 		I2Cx --> the I2C peripheral e.g. I2C1
 * 		address --> the 7 bit slave address
 * 		direction --> the tranmission direction can be:
 * 						I2C_Direction_Tranmitter for Master transmitter mode
 * 						I2C_Direction_Receiver for Master receiver
 */
static void I2C_start(I2C_TypeDef* I2Cx, rt_uint8_t address, rt_uint8_t direction)
{
	// wait until I2C1 is not busy anymore
	while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));
  
	// Send I2C1 START condition by setting the START bit.
	I2C_GenerateSTART(I2Cx, ENABLE);
	  
	// wait for I2C1 EV5 --> Slave has acknowledged start condition
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));
		
	// Send slave Address for write 
	I2C_Send7bitAddress(I2Cx, address, direction);
	  
	/* wait for I2C1 EV6, check if 
	 * either Slave has acknowledged Master transmitter or
	 * Master receiver mode, depending on the transmission
	 * direction
	 */ 
	if(direction == I2C_Direction_Transmitter){
		while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	}
	else if(direction == I2C_Direction_Receiver){
		while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
	}
}


/* This function transmits one byte to the slave device
 * Parameters:
 *		I2Cx --> the I2C peripheral e.g. I2C1 
 *		data --> the data byte to be transmitted
 */
static void I2C_write(I2C_TypeDef* I2Cx, rt_uint8_t data)
{
	I2C_SendData(I2Cx, data);
	// wait for I2C1 EV8_2 --> byte has been transmitted
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
}


/* This function reads one byte from the slave device 
 * and acknowledges the byte (requests another byte)
 */
static rt_uint8_t I2C_read_ack(I2C_TypeDef* I2Cx)
{
	// enable acknowledge of recieved data
	I2C_AcknowledgeConfig(I2Cx, ENABLE);
	// wait until one byte has been received
	while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) );
	// read data from I2C data register and return data byte
	return (rt_uint8_t)I2C_ReceiveData(I2Cx);
}


/* This function reads one byte from the slave device
 * and doesn't acknowledge the recieved data 
 */
static rt_uint8_t I2C_read_nack(I2C_TypeDef* I2Cx)
{
    rt_base_t level;
    rt_uint8_t result;
    rt_uint16_t cnt = 0;

    
    // Note: access of the I2C bus here is protected by shutting down
    // the IRQs and the scheduler, so that the accessing process
    // will not be interrupted. Bad things happen if I don't do this.
    // (Inspired byAN2824 from ST)
    __disable_irq();
    level = rt_hw_interrupt_disable();
    
	// disabe acknowledge of received data
	// nack also generates stop condition after last byte received
	// see reference manual for more info        
	I2C_AcknowledgeConfig(I2Cx, DISABLE);
	I2C_GenerateSTOP(I2Cx, ENABLE);

	// wait until one byte has been received
	while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) )
    {
        if (cnt++ > 1000) 
        {
            cnt = 0;
            break;
        }
    }
    
	// read data from I2C data register and return data byte
	result = (rt_uint8_t) I2C_ReceiveData(I2Cx);
    
    rt_hw_interrupt_enable(level);
    __enable_irq();
    return result;
}


/* This funtion issues a stop condition and therefore
 * releases the bus
 */
static void I2C_stop(I2C_TypeDef* I2Cx)
{
	// Send I2C1 STOP Condition 
	I2C_GenerateSTOP(I2Cx, ENABLE);
}


static void EXTI_Configuration(void)
{
    EXTI_InitTypeDef EXTI_InitStruct;
    
    // connect the RDY pin to its EXTI line (Note: hard-coded port and pin value!!!)
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource0);
    
    EXTI_InitStruct.EXTI_Line = EXTI_Line0;  // RDY is on PC0
    EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;  // active low
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;
    
    EXTI_Init(&EXTI_InitStruct);
}


static void NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStruct;
    
    NVIC_InitStruct.NVIC_IRQChannel = EXTI0_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    
    NVIC_Init(&NVIC_InitStruct);
}


static void RCC_Configuration(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    // enable GPIOB clock
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    // enable GPIOC clock
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    // enable SYSCFG clock so as to write to SYSCFG
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
}


static void GPIO_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    
    // configure pins used by I2C1
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD; // GPIO set to open-drain
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	// connect I2C1 pins to I2C alternate function
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_I2C1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1);
    
    // configure PC0 to receive the RDY interrupt from ADS1115
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
    GPIO_Init(GPIOC, &GPIO_InitStruct);
}


static void I2C1_Configuration(void)
{
	I2C_InitTypeDef I2C_InitStruct;
    
    // configure I2C1 
	I2C_InitStruct.I2C_ClockSpeed = MY_I2C_CLOCK_RATE; 		// 10kHz
	I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;			// I2C mode
	I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;	// 50% duty cycle --> standard
	I2C_InitStruct.I2C_OwnAddress1 = 0x00;			// own address, not relevant in master mode
    // disable acknowledge when reading (can be changed later on)
	I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;		
    // set address length to 7 bit addresses
	I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; 
    
	I2C_Init(I2C1, &I2C_InitStruct);				// init I2C1
    I2C_Cmd(I2C1, ENABLE);                          // enable I2C1
}