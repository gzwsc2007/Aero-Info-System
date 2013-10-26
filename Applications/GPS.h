#ifndef _GPS_H_
#define _GPS_H_

// USART Settings for receiving GPS Data 
#define GPS_USART              USART1
#define GPS_USART_BAUD_RATE    115200
#define GPS_BUF_LEN            100

typedef struct {
    // Navigation succeeded or not.
    rt_bool_t  valid;
    // Geographic position. Encoded as an integer in the format
    // d d m m . m m m m m (degree & minute)
    rt_int32_t latitude;   // positive - N , negative - S
    // d d d m m . m m m m m 
    rt_int32_t longitude;  // positive - E , negative - W
    // speed in knot, encoded in integer as ssss (meaning sss.s)
    rt_int16_t speed;
    // course in degree (0 deg corresponds to geographic North).
    // Encoded in integer as cccc (meaning ccc.c)
    rt_int16_t course;
} GPS_Data_t;
    

// Global flag indicating whether the GPS data struct is being
// accessed. Before any access to the GPS_struct, one should set
// this busy flag, so that the parser will not modifiy the struct.
extern rt_bool_t   GPS_struct_busy;
extern GPS_Data_t  gps;

void GPS_Init(void);

#endif
