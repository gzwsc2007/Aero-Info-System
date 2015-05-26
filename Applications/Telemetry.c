#include "stm32f4xx.h"
#include <rtthread.h>
#include "NRF24L01.h"
#include "GPS.h"
#include "I2CSensors.h"
#include "MPU6050.h"
#include "Telemetry.h"
#include "MAVlink_include/aeroInfoSystem/mavlink.h"
#include "rwlock.h"

#define MY_SYS_ID   250

struct rt_event event_drdy; // global event - data ready

static mavlink_pfd_t mavlinkPFDStruct; // static global mavlink PFD struct (custom message)
static mavlink_navd_t mavlinkNavDStruct;

static rt_int32_t pres;
static rt_int16_t temp;

/* update the static global mavlink PFD struct with new values. Does nothing
   if some other dependent structs are busy (e.g. the MPUData struct)

   NOTE: This function must be called inside a thread (as it uses semaphores)
*/
static void mavlink_updatePFDStruct(void)
{
    if (MPU_struct_busy)
        return;
    
    // gather attitude info
    MPU_struct_busy = RT_TRUE;
    mavlinkPFDStruct.roll = (int16_t)(MPU_Data.roll * 100.0);
    mavlinkPFDStruct.pitch = (int16_t)(MPU_Data.pitch * 100.0);
    mavlinkPFDStruct.yaw = (int16_t)(MPU_Data.yaw * 100.0); // may wanna use Magnetometer in the future
    MPU_struct_busy = RT_FALSE;
    // gather altitude info
    BMP085_getTempAndPres(&temp, &pres);
    mavlinkPFDStruct.altitude = (int16_t)(BMP085_presToAlt(pres) * 10.0);
    mavlinkNavDStruct.temp = (int16_t)temp;
    // gather airspeed info
    mavlinkPFDStruct.airspeed = (int16_t)(getAirspeed() * 10.0);
    // gather battery current info
    mavlinkPFDStruct.battI = (int16_t)(getBattCurrent() * 100); // in 0.01 A
}

static void mavlink_updateNavDStruct(void)
{
    if (GPS_struct_busy)
        return;
    
    mavlinkNavDStruct.latitude = (int32_t)GPS_Data.latitude;
    mavlinkNavDStruct.longitude = (int32_t)GPS_Data.longitude;
    mavlinkNavDStruct.course = (uint16_t)GPS_Data.course;
    mavlinkNavDStruct.groundspeed = (uint16_t)GPS_Data.speed;
    
    // battery info
    mavlinkNavDStruct.battV = (int16_t)(getBattVoltage() * 100); // in 0.01 Volt
}

void mavlink_thread_entry(void *parameter) 
{
    rt_uint8_t cnt = 0; // counter used to generate 5Hz sending rate.
    
    rt_uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    rt_uint16_t len;
    mavlink_message_t msg;

    // not gonna use this event yet
    if (rt_event_init(&event_drdy, "drdy", RT_IPC_FLAG_FIFO) != RT_EOK)
        rt_kprintf("event_drdy init error\n");
    
    while(1)
    {
    /* Send PFD data at 25 Hz*/
        // fill the pfd struct
        mavlink_updatePFDStruct();
        
        // pack the pfd message
        mavlink_msg_pfd_pack(MY_SYS_ID, 
                             200, // component ID - IMU
                             &msg,
                             mavlinkPFDStruct.roll,
                             mavlinkPFDStruct.pitch,
                             mavlinkPFDStruct.yaw,
                             mavlinkPFDStruct.altitude,
                             mavlinkPFDStruct.airspeed,
                             mavlinkPFDStruct.battI);
        
        // copy the message to the send buffer
        len = mavlink_msg_to_send_buffer(buf, &msg);
        // "Send" the message
        radioSendWrapper(buf, len);
        
        
        /* Send NavD data at 5 Hz */
        if (cnt >= 3)
        {
            cnt = 0;
            
            // fill the NavD struct
            mavlink_updateNavDStruct();
                
            // pack the NavD message
            mavlink_msg_navd_pack(MY_SYS_ID, 
                             200, // component ID - IMU
                             &msg,
                             mavlinkNavDStruct.battV,
                             mavlinkNavDStruct.temp,
                             mavlinkNavDStruct.latitude,
                             mavlinkNavDStruct.longitude,
                             mavlinkNavDStruct.course,
                             mavlinkNavDStruct.groundspeed);

            // copy the message to the send buffer
            len = mavlink_msg_to_send_buffer(buf, &msg);
            // "Send" the message
            radioSendWrapper(buf, len);
            
            rt_thread_delay(10); // delay for a shorter time
            continue;
        }
        
        cnt++;
        rt_thread_delay(25);
    }
}


/*
 * A wrapper for the NRF24_Send function. Automatically adds the length of 
 * each packet to the beginning. For data more than 32 bytes, this function
 * will truncate it into multiple packets and send them sequentially via the
 * NRF24_Send function.
 */
void radioSendWrapper(rt_uint8_t *data, rt_uint32_t len) 
{
    rt_uint8_t packetLen = 0;
    rt_uint32_t dataPtr = 0;
    rt_uint8_t buf[32];
    
    while (len > 31)
    {
        // assemble the packet
        packetLen = 31;
        buf[0] = packetLen;
        memcpy(buf + 1, data + dataPtr, packetLen);
        
        NRF24_Send(buf, 32, RT_FALSE);
        len -= 31;
        dataPtr += 31;
    }
    
    // handle the rest of the data
    if (len > 0) {
        memset(buf, 0, 32);
        packetLen = len;
        buf[0] = packetLen;
        memcpy(buf + 1, data + dataPtr, packetLen);

        NRF24_Send(buf, 32, RT_FALSE);
    }
}