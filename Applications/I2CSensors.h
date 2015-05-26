#ifndef _I2CSENSORS_H_
#define  _I2CSENSORS_H_

#define MY_I2C_CLOCK_RATE  50000 // 50 kHz for now. Should increase in future.

// R1 = 17.71 kohm = 17710 ohm
// R2 = 5017 ohm + 2161 ohm = 7178 ohm
#define VBATT_SCALING_FACTOR 3.467261 //  (R1 + R2) / R2

#define ADC_CHAN_VBATT      1 // ADC channel for measuring LiPo Batt voltage
#define ADC_CHAN_VCURRENT   3 // ADC channel for measuring battery current
#define ADC_CHAN_VAIRSPEED  2 // ADC channel for measruing airspeed sensor
#define ADC_CHAN_VDD        3 // ADC channel for measuring the 5V power supply

#define BMP085_OSS          0x03 // oversampling setting (3 is high-resolution)
#define BMP085_ADDR         0xEE // shifted 7-bit address. Lowest bit always 0
#define BMP085_CTRL_REG     0xF4 // address of the control register
#define BMP085_MSB_REG      0xF6
#define BMP085_LSB_REG      0xF7
#define BMP085_XLSB_REG     0xF8
#define BMP085_READ_UT      0x2E // command for starting temperature conversion (put in reg 0xf4)
#define BMP085_READ_UP      0x34 // command for starting pressure conversion

#define HMC5883_ADDR        0x3C // shifted 7-bit address. Lowest bit always 0
#define HMC5883_CRA_REG     0x00
#define HMC5883_CRB_REG     0x01
#define HMC5883_MR_REG      0x02
#define HMC5883_IRA_REG     0x0A // Identification Register A (reads "H")
#define HMC5883_IRB_REG     0x0B // reads "4"
#define HMC5883_IRC_REG     0x0C // reads "3"

/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
#define ADS1115_ADDR                    (0x90)    // 1001 0000 (ADDR = GND)
/*=========================================================================*/

/*=========================================================================
    POINTER REGISTER
-----------------------------------------------------------------------*/
#define ADS1115_REG_POINTER_MASK        (0x03)
#define ADS1115_REG_POINTER_CONVERT     (0x00)
#define ADS1115_REG_POINTER_CONFIG      (0x01)
#define ADS1115_REG_POINTER_LOWTHRESH   (0x02)
#define ADS1115_REG_POINTER_HITHRESH    (0x03)
/*=========================================================================*/

/*=========================================================================
    CONFIG REGISTER
-----------------------------------------------------------------------*/
#define ADS1115_REG_CONFIG_OS_MASK      (0x8000)
#define ADS1115_REG_CONFIG_OS_SINGLE    (0x8000)  // Write: Set to start a single-conversion
#define ADS1115_REG_CONFIG_OS_BUSY      (0x0000)  // Read: Bit = 0 when conversion is in progress
#define ADS1115_REG_CONFIG_OS_NOTBUSY   (0x8000)  // Read: Bit = 1 when device is not performing a conversion

#define ADS1115_REG_CONFIG_MUX_MASK     (0x7000)
#define ADS1115_REG_CONFIG_MUX_DIFF_0_1 (0x0000)  // Differential P = AIN0, N = AIN1 (default)
#define ADS1115_REG_CONFIG_MUX_DIFF_0_3 (0x1000)  // Differential P = AIN0, N = AIN3
#define ADS1115_REG_CONFIG_MUX_DIFF_1_3 (0x2000)  // Differential P = AIN1, N = AIN3
#define ADS1115_REG_CONFIG_MUX_DIFF_2_3 (0x3000)  // Differential P = AIN2, N = AIN3
#define ADS1115_REG_CONFIG_MUX_SINGLE_0 (0x4000)  // Single-ended AIN0
#define ADS1115_REG_CONFIG_MUX_SINGLE_1 (0x5000)  // Single-ended AIN1
#define ADS1115_REG_CONFIG_MUX_SINGLE_2 (0x6000)  // Single-ended AIN2
#define ADS1115_REG_CONFIG_MUX_SINGLE_3 (0x7000)  // Single-ended AIN3

#define ADS1115_REG_CONFIG_PGA_MASK     (0x0E00)
#define ADS1115_REG_CONFIG_PGA_6_144V   (0x0000)  // +/-6.144V range = Gain 2/3
#define ADS1115_REG_CONFIG_PGA_4_096V   (0x0200)  // +/-4.096V range = Gain 1
#define ADS1115_REG_CONFIG_PGA_2_048V   (0x0400)  // +/-2.048V range = Gain 2 (default)
#define ADS1115_REG_CONFIG_PGA_1_024V   (0x0600)  // +/-1.024V range = Gain 4
#define ADS1115_REG_CONFIG_PGA_0_512V   (0x0800)  // +/-0.512V range = Gain 8
#define ADS1115_REG_CONFIG_PGA_0_256V   (0x0A00)  // +/-0.256V range = Gain 16

#define ADS1115_REG_CONFIG_MODE_MASK    (0x0100)
#define ADS1115_REG_CONFIG_MODE_CONTIN  (0x0000)  // Continuous conversion mode
#define ADS1115_REG_CONFIG_MODE_SINGLE  (0x0100)  // Power-down single-shot mode (default)

#define ADS1115_REG_CONFIG_DR_MASK     (0x00E0)  
#define ADS1115_REG_CONFIG_DR_8SPS     (0x0000)  // 8 samples per second
#define ADS1115_REG_CONFIG_DR_16SPS    (0x0020)  // 16 samples per second
#define ADS1115_REG_CONFIG_DR_32SPS    (0x0040)  // 32 samples per second
#define ADS1115_REG_CONFIG_DR_64SPS    (0x0060)  // 64 samples per second
#define ADS1115_REG_CONFIG_DR_128SPS   (0x0080)  // 128 samples per second (default)
#define ADS1115_REG_CONFIG_DR_250SPS   (0x00A0)  // 250 samples per second
#define ADS1115_REG_CONFIG_DR_475SPS   (0x00C0)  // 475 samples per second
#define ADS1115_REG_CONFIG_DR_860SPS   (0x00E0) // 860 samples per second

#define ADS1115_REG_CONFIG_CMODE_MASK   (0x0010)
#define ADS1115_REG_CONFIG_CMODE_TRAD   (0x0000)  // Traditional comparator with hysteresis (default)
#define ADS1115_REG_CONFIG_CMODE_WINDOW (0x0010)  // Window comparator

#define ADS1115_REG_CONFIG_CPOL_MASK    (0x0008)
#define ADS1115_REG_CONFIG_CPOL_ACTVLOW (0x0000)  // ALERT/RDY pin is low when active (default)
#define ADS1115_REG_CONFIG_CPOL_ACTVHI  (0x0008)  // ALERT/RDY pin is high when active

#define ADS1115_REG_CONFIG_CLAT_MASK    (0x0004)  // Determines if ALERT/RDY pin latches once asserted
#define ADS1115_REG_CONFIG_CLAT_NONLAT  (0x0000)  // Non-latching comparator (default)
#define ADS1115_REG_CONFIG_CLAT_LATCH   (0x0004)  // Latching comparator

#define ADS1115_REG_CONFIG_CQUE_MASK    (0x0003)
#define ADS1115_REG_CONFIG_CQUE_1CONV   (0x0000)  // Assert ALERT/RDY after one conversions
#define ADS1115_REG_CONFIG_CQUE_2CONV   (0x0001)  // Assert ALERT/RDY after two conversions
#define ADS1115_REG_CONFIG_CQUE_4CONV   (0x0002)  // Assert ALERT/RDY after four conversions
#define ADS1115_REG_CONFIG_CQUE_NONE    (0x0003)  // Disable the comparator and put ALERT/RDY in high state (default)

      
typedef enum
{
  GAIN_TWOTHIRDS    = ADS1115_REG_CONFIG_PGA_6_144V,
  GAIN_ONE          = ADS1115_REG_CONFIG_PGA_4_096V,
  GAIN_TWO          = ADS1115_REG_CONFIG_PGA_2_048V,
  GAIN_FOUR         = ADS1115_REG_CONFIG_PGA_1_024V,
  GAIN_EIGHT        = ADS1115_REG_CONFIG_PGA_0_512V,
  GAIN_SIXTEEN      = ADS1115_REG_CONFIG_PGA_0_256V
} adsGain_t;

// return battery current in Amps
double getBattCurrent(void);

// return battery voltage in volts
double getBattVoltage(void);

// reset the neutral point for airspeed voltage
void calibrateAirspeed(void);

/* return airspeed in m/s */
double getAirspeed(void);

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
void ADS1115_thread_entry(void *param);


/*
 * Responsible for gathering data from the BMP085 barometer constantly, and 
 * convert the readings into temperature (0.01 C), pressure (Pa) and relative 
 * altitude (m)
 *
* Refreshing rate:
*  - Temperature: 1 - 2 Hz
*  - Pressure (Altitude): 20 Hz
 */
void BMP085_thread_entry(void *parameters);


/* Initialize the I2C sensors */
void I2CSensors_Init(void);

/*
 * Read the converted voltage from a designated channel in the ADS1115.
 */
rt_int16_t ADS1115_readSingleEnded(rt_uint8_t channel);

/*
 * Set the PGA gain of the ADS1115. The input parameter must be of the
 * adsGain_t enum type
 */
void ADS1115_setGain(adsGain_t gain);

/*
 * Return the current PGA gain setting.
 */
adsGain_t ADS1115_getGain(void);


// calculate the relative altitude (wrt to ground level)
float BMP085_presToAlt(rt_int32_t pres);

/*
 * main Interface function
 *
 * Return the measured temperature and pressure into the pointers provided.
 * The unit of temp is in 0.1 Celcius and the unit for pres is in Pa.
 * 
 * This function is non-blocking. The data gathering process is handled in the background.
 */
void BMP085_getTempAndPres(rt_int16_t *temp, rt_int32_t *pres);


/*
 * Read the raw pressure data and convert it to real pressure in Pa
 *
 * Note: the variable "b5" is required in this function. So BMP085_getTemperature(..) must
 * be called prior to this function.
 *
 * Source: https://www.sparkfun.com/tutorials/253
 */
rt_int32_t BMP085_getPressure(void);


/*
 * Read the raw temperature data from the sensor and then convert it to real temperature
 * in 0.1 Celcius
 *
 * Source: https://www.sparkfun.com/tutorials/253
 */
rt_int16_t BMP085_getTemperature(void);

#endif