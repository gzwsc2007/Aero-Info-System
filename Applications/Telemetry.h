#ifndef _TELEMETRY_H_
#define _TELEMETRY_H_

/* global event - data ready: as soon as a sensor collects its data, a "data
 * ready" event should be published so that the mavlink thread could send the
 * data to ground in a timely manner. The events are defined in the macros below
 */
extern struct rt_event event_drdy;

#define EVENT_GPS_DATA_RDY          (1)
#define EVENT_ATTITUDE_DATA_RDY     (1 << 1)
#define EVENT_BATTERY_STATUS_RDY    (1 << 2)
#define EVENT_VFR_HUD_DATA_RDY      (1 << 3)
#define EVENT_ALL                   (0xFFFFFFFF)

void mavlink_thread_entry(void *params);

/*
 * A wrapper for the NRF24_Send function. Automatically adds the length of 
 * each packet to the beginning. For data more than 32 bytes, this function
 * will truncate it into multiple packets and send them sequentially via the
 * NRF24_Send function.
 */
void radioSendWrapper(rt_uint8_t *data, rt_uint32_t len);

#endif