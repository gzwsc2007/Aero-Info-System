#ifndef _NRF24L01_H_
#define _NRF24L01_H_

#define NRF24_CE_HIGH() GPIO_SetBits(NRF24_CE_PORT, NRF24_CE_PIN)
#define NRF24_CE_LOW()  GPIO_ResetBits(NRF24_CE_PORT, NRF24_CE_PIN)

#define NRF24_CSN_LOW()  GPIO_ResetBits(NRF24_CS_PORT, NRF24_CS_PIN)  // pull CS low
#define NRF24_CSN_HIGH() GPIO_SetBits(NRF24_CS_PORT, NRF24_CS_PIN)  // pull CS high 

// SPI port used for NRF24L01
#define NRF24_SPI_PORT  SPI1
// The CS pin for SPI
#define NRF24_CS_PORT   GPIOA
#define NRF24_CS_PIN    GPIO_Pin_4
// CE pin for NRF24L01
#define NRF24_CE_PORT   GPIOC
#define NRF24_CE_PIN    GPIO_Pin_4
// IRQ pin for NRF24L01
#define NRF24_IRQ_PORT  GPIOC
#define NRF24_IRQ_PIN   GPIO_Pin_5

// Default device config: 1 byte CRC
#define NRF24_DEFAULT_CONFIGURATION (NRF24_EN_CRC | NRF24_CRCO)

// This is the maximum message length that can be supported by this library. Limited by
// the suported message lengths oin the nRF24
// Can be pre-defined to a smaller size (to save SRAM) prior to including this header
#ifndef NRF24_MAX_MESSAGE_LEN
#define NRF24_MAX_MESSAGE_LEN 32
#endif

// Defines convenient values for setting data rates in setRF()
typedef enum
{
	NRF24DataRate1Mbps = 0,   ///< 1 Mbps
	NRF24DataRate2Mbps,       ///< 2 Mbps
	NRF24DataRate250kbps      ///< 250 kbps
} NRF24DataRate;

/// Convenient values for setting transmitter power in setRF()
/// These are designed to agree with the values for RF_PWR
/// To be passed to setRF();
typedef enum
{
	NRF24TransmitPowerm18dBm = 0,   ///< -18 dBm
	NRF24TransmitPowerm12dBm,       ///< -12 dBm
	NRF24TransmitPowerm6dBm,        ///< -6 dBm
	NRF24TransmitPower0dBm          ///< 0 dBm
} NRF24TransmitPower;

// SPI Command names
#define NRF24_COMMAND_R_REGISTER                        0x00
#define NRF24_COMMAND_W_REGISTER                        0x20
#define NRF24_COMMAND_R_RX_PAYLOAD                      0x61
#define NRF24_COMMAND_W_TX_PAYLOAD                      0xa0
#define NRF24_COMMAND_FLUSH_TX                          0xe1
#define NRF24_COMMAND_FLUSH_RX                          0xe2
#define NRF24_COMMAND_REUSE_TX_PL                       0xe3
#define NRF24_COMMAND_ACTIVATE                          0x50
#define NRF24_COMMAND_R_RX_PL_WID                       0x60
#define NRF24_COMMAND_W_ACK_PAYLOAD                     0xa8
#define NRF24_COMMAND_W_TX_PAYLOAD_NOACK                0xb0
#define NRF24_COMMAND_NOP                               0xff

// Register names
#define NRF24_REGISTER_MASK                             0x1f
#define NRF24_REG_00_CONFIG                             0x00
#define NRF24_REG_01_EN_AA                              0x01
#define NRF24_REG_02_EN_RXADDR                          0x02
#define NRF24_REG_03_SETUP_AW                           0x03
#define NRF24_REG_04_SETUP_RETR                         0x04
#define NRF24_REG_05_RF_CH                              0x05
#define NRF24_REG_06_RF_SETUP                           0x06
#define NRF24_REG_07_STATUS                             0x07
#define NRF24_REG_08_OBSERVE_TX                         0x08
#define NRF24_REG_09_RPD                                0x09
#define NRF24_REG_0A_RX_ADDR_P0                         0x0a
#define NRF24_REG_0B_RX_ADDR_P1                         0x0b
#define NRF24_REG_0C_RX_ADDR_P2                         0x0c
#define NRF24_REG_0D_RX_ADDR_P3                         0x0d
#define NRF24_REG_0E_RX_ADDR_P4                         0x0e
#define NRF24_REG_0F_RX_ADDR_P5                         0x0f
#define NRF24_REG_10_TX_ADDR                            0x10
#define NRF24_REG_11_RX_PW_P0                           0x11
#define NRF24_REG_12_RX_PW_P1                           0x12
#define NRF24_REG_13_RX_PW_P2                           0x13
#define NRF24_REG_14_RX_PW_P3                           0x14
#define NRF24_REG_15_RX_PW_P4                           0x15
#define NRF24_REG_16_RX_PW_P5                           0x16
#define NRF24_REG_17_FIFO_STATUS                        0x17
#define NRF24_REG_1C_DYNPD                              0x1c
#define NRF24_REG_1D_FEATURE                            0x1d

// These register masks etc are named wherever possible
// corresponding to the bit and field names in the nRF24L01 Product Specification
// #define NRF24_REG_00_CONFIG                             0x00
#define NRF24_MASK_RX_DR                                0x40
#define NRF24_MASK_TX_DS                                0x20
#define NRF24_MASK_MAX_RT                               0x10
#define NRF24_EN_CRC                                    0x08
#define NRF24_CRCO                                      0x04
#define NRF24_PWR_UP                                    0x02
#define NRF24_PRIM_RX                                   0x01

// #define NRF24_REG_01_EN_AA                              0x01
#define NRF24_ENAA_P5                                   0x20
#define NRF24_ENAA_P4                                   0x10
#define NRF24_ENAA_P3                                   0x08
#define NRF24_ENAA_P2                                   0x04
#define NRF24_ENAA_P1                                   0x02
#define NRF24_ENAA_P0                                   0x01

// #define NRF24_REG_02_EN_RXADDR                          0x02
#define NRF24_ERX_P5                                    0x20
#define NRF24_ERX_P4                                    0x10
#define NRF24_ERX_P3                                    0x08
#define NRF24_ERX_P2                                    0x04
#define NRF24_ERX_P1                                    0x02
#define NRF24_ERX_P0                                    0x01

// #define NRF24_REG_03_SETUP_AW                           0x03
#define NRF24_AW_3_BYTES                                0x01
#define NRF24_AW_4_BYTES                                0x02
#define NRF24_AW_5_BYTES                                0x03

// #define NRF24_REG_04_SETUP_RETR                         0x04
#define NRF24_ARD                                       0xf0
#define NRF24_ARC                                       0x0f

// #define NRF24_REG_05_RF_CH                              0x05
#define NRF24_RF_CH                                     0x7f

// #define NRF24_REG_06_RF_SETUP                           0x06
#define NRF24_CONT_WAVE                                 0x80
#define NRF24_RF_DR_LOW                                 0x20
#define NRF24_PLL_LOCK                                  0x10
#define NRF24_RF_DR_HIGH                                0x08
#define NRF24_PWR                                       0x07
#define NRF24_PWR_m18dBm                                0x00
#define NRF24_PWR_m12dBm                                0x02
#define NRF24_PWR_m6dBm                                 0x04
#define NRF24_PWR_0dBm                                  0x06

// #define NRF24_REG_07_STATUS                             0x07
#define NRF24_RX_DR                                     0x40
#define NRF24_TX_DS                                     0x20
#define NRF24_MAX_RT                                    0x10
#define NRF24_RX_P_NO                                   0x0e
#define NRF24_TX_FULL                                   0x01


// #define NRF24_REG_08_OBSERVE_TX                         0x08
#define NRF24_PLOS_CNT                                  0xf0
#define NRF24_ARC_CNT                                   0x0f

// #define NRF24_REG_09_RPD                                0x09
#define NRF24_RPD                                       0x01

// #define NRF24_REG_17_FIFO_STATUS                        0x17
#define NRF24_FIFO_TX_REUSE                             0x40
#define NRF24_FIFO_TX_FULL                              0x20
#define NRF24_FIFO_TX_EMPTY                             0x10
#define NRF24_FIFO_RX_FULL                              0x02
#define NRF24_FIFO_RX_EMPTY                             0x01

// #define NRF24_REG_1C_DYNPD                              0x1c
#define NRF24_DPL_P5                                    0x20
#define NRF24_DPL_P4                                    0x10
#define NRF24_DPL_P3                                    0x08
#define NRF24_DPL_P2                                    0x04
#define NRF24_DPL_P1                                    0x02
#define NRF24_DPL_P0                                    0x01

// #define NRF24_REG_1D_FEATURE                            0x1d
#define NRF24_EN_DPL                                    0x04
#define NRF24_EN_ACK_PAY                                0x02
#define NRF24_EN_DYN_ACK                                0x01


/****************** interfaces *********************/

/* Data structure for the RX and TX buffer */
struct NRF24_Buf_Struct {
    rt_uint8_t data[32];
    rt_uint8_t len;  // effective length
};

/*
 * Contains the RX data read from the radio, as well as the effective length of the data.
 *
 * There are two fields in the struct:
 * data - a 32-byte array
 * len - the effective length of the meassage
 *
 * Caution: Client should not access this struct unless it is clear that a valid message
 *          has been received. (len != 0)
 */
extern struct NRF24_Buf_Struct NRF24_RX_Buf;

/*
 * Contains the TX data to be sent through the radio. Client should provide the effective
 * length of the data in the len field.
 *
 * Note: Access to this buffer should be protected by mutex lock.
 */
extern struct NRF24_Buf_Struct NRF24_TX_Buf;

/* 
 * Initialize the NRF24L01 radio module 
 */
void NRF24_Init(void);

/*
 * The Radio_Thread, responsible for transmitting and receiving messages at a constant
 * period (about 25 times per second).
 */
void radio_thread_entry(void *parameter);

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
rt_uint8_t NRF24_Send(rt_uint8_t *data, rt_uint8_t len, rt_bool_t noack);

/*
 * Check if there is a valid payload in the RX_FIFO of the radio. If yes, copy it
 * to buf and set *len to the size of the copied payload (must be <= 32 bytes, so
 * buf should be a buffer with 32 bytes). If not, simply return false.
 *
 * Note: The radio should be configured into RX mode before calling this function.
 *
 * Return RT_TRUE if a payload is copied into buf, and RT_FALSE otherwise.
 */
rt_bool_t NRF24_Recv(rt_uint8_t *buf, rt_uint8_t *len);


void printRegisters(void);

#endif
