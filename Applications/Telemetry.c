#include "stm32f4xx.h"
#include <rtthread.h>
#include "NRF24L01.h"
#include "GPS.h"
#include "I2CSensors.h"
#include "MPU6050.h"
#include "Telemetry.h"
#include "MAVlink_include/common/mavlink.h"
#include "rwlock.h"

#define MY_SYS_ID   250

struct rt_event event_drdy; // global event - data ready


void mavlink_thread_entry(void *parameter) 
{
    rt_tick_t tlast = 0;
    rt_tick_t tnow = 0;
    rt_uint32_t e; // temp var for receiving event
    
    rt_uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    rt_uint16_t len;
    mavlink_message_t msg;
    mavlink_attitude_t mav_att;
    mavlink_gps_raw_int_t mav_gps;
    
    if (rt_event_init(&event_drdy, "drdy", RT_IPC_FLAG_FIFO) != RT_EOK)
        rt_kprintf("event_drdy init error\n");
    
    while(1)
    {
        // update system time
        tnow = rt_tick_get();
        
        /* Send Heartbeat #0 everyr 2 seconds*/
        if (tnow - tlast > 2000)
        {
            tlast = tnow;
            
            // pack the heart-beat message
            mavlink_msg_heartbeat_pack(MY_SYS_ID, 
                                       MAV_COMP_ID_IMU,
                                       &msg,
                                       MAV_TYPE_FIXED_WING,
                                       MAV_AUTOPILOT_ARDUPILOTMEGA,
                                       MAV_MODE_AUTO_ARMED, // system mode
                                       0, // custom mode
                                       MAV_STATE_ACTIVE);
            // copy the message to the send buffer
            len = mavlink_msg_to_send_buffer(buf, &msg);
            // "Send" the message
            radioSendWrapper(buf, len);
        }
        
        /* Wait for the data ready event to arrive */
        if (rt_event_recv(&event_drdy, EVENT_ALL, RT_EVENT_FLAG_OR | 
                          RT_EVENT_FLAG_CLEAR, 1000, &e) == RT_EOK)
        {
            /* Send Status */
            if (e & EVENT_BATTERY_STATUS_RDY) 
            {
                mavlink_msg_sys_status_pack(MY_SYS_ID, 
                                            MAV_COMP_ID_IMU, 
                                            &msg, 
                                            0, 0, 0, // dont care
                                            500,   // load
                                            11000, // volt in 1 mV
                                            -1, // current in 10 mA
                                            -1, 0, 0, 0, 0, 0, 0);
                len = mavlink_msg_to_send_buffer(buf, &msg);
                radioSendWrapper(buf, len);
            }
            
            /* Send Attitude #30 */
            if (e & EVENT_ATTITUDE_DATA_RDY) 
            {
                MPU_fillMavlinkStruct(&mav_att);      
                mavlink_msg_attitude_pack(MY_SYS_ID,
                                          MAV_COMP_ID_IMU,
                                          &msg,
                                          mav_att.time_boot_ms,
                                          mav_att.roll,
                                          mav_att.pitch,
                                          mav_att.yaw,
                                          mav_att.rollspeed,
                                          mav_att.pitchspeed,
                                          mav_att.yawspeed);
                len = mavlink_msg_to_send_buffer(buf, &msg);
                radioSendWrapper(buf, len);
            }
           
            /* Send GPS raw int #24 */
            if (e & EVENT_GPS_DATA_RDY) 
            {
                GPS_fillMavlinkStruct(&mav_gps);
                // fix_type == 0 means GPS data invalid
                if (mav_gps.fix_type != 0) 
                {
                    mavlink_msg_gps_raw_int_pack(MY_SYS_ID,
                                                 MAV_COMP_ID_IMU,
                                                 &msg,
                                                 mav_gps.time_usec,
                                                 mav_gps.fix_type,
                                                 mav_gps.lat,
                                                 mav_gps.lon,
                                                 mav_gps.alt,
                                                 mav_gps.eph,
                                                 mav_gps.epv,
                                                 mav_gps.vel,
                                                 mav_gps.cog,
                                                 mav_gps.satellites_visible);
                    len = mavlink_msg_to_send_buffer(buf, &msg);
                    radioSendWrapper(buf, len);
                }
            }
            
            /* Send VFR_HUD #74 */
            if (e & EVENT_VFR_HUD_DATA_RDY)
            {
                
            }
        }
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
    rt_uint8_t dataPtr = 0;
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