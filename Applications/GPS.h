#ifndef _GPS_H_
#define _GPS_H_

#include "MAVlink_include/common/mavlink.h"

// USART Settings for receiving GPS Data 
#define GPS_USART              USART1
#define GPS_USART_BAUD_RATE    115200
#define GPS_BUF_LEN            100


/* 
 * Data in this strucct is compatible with the MAVlink protocol
 */
typedef struct {
    // Navigation succeeded or not.
    rt_bool_t  valid;
    // Geographic position. Encoded as an integer with unit
    // 10^-7 degree
    rt_int32_t latitude;   // positive - N , negative - S
    // Encoded as an integer with unit 10^-7 degree
    rt_int32_t longitude;  // positive - E , negative - W
    // speed in 0.01 m/s
    rt_uint16_t speed;
    // course in degree (0 deg corresponds to geographic North).
    // Encoded with unit 0.01 deg
    rt_uint16_t course;
} GPS_Data_t;
    

// Global flag indicating whether the GPS data struct is being
// accessed. Before any access to the GPS_struct, one should set
// this busy flag, so that the parser will not modifiy the struct.
extern rt_bool_t   GPS_struct_busy;
// Global structure that stores interpreted gps information
extern GPS_Data_t  GPS_Data;

void GPS_Init(void);

void GPS_fillMavlinkStruct(mavlink_gps_raw_int_t *m);

#endif
