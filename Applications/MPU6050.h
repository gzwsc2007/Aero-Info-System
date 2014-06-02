#ifndef _MPU6050_H_
#define _MPU6050_H_

#define MPU_USART               USART3
#define MPU_USART_BAUD_RATE     115200
#define MPU_BUF_LEN             40

typedef struct { // units are degrees
    float roll;
    float pitch;
    float yaw;
    float rollspeed;
    float pitchspeed;
    float yawspeed;
    float temperature;
} MPU_Data_t;

extern MPU_Data_t MPU_Data;
extern rt_bool_t MPU_struct_busy;

void MPU6050_Init(void);

#endif