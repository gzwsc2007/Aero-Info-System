// MESSAGE NavD PACKING

#define MAVLINK_MSG_ID_NavD 151

typedef struct __mavlink_navd_t
{
 int32_t latitude; ///< Latitude (WGS84), in 10^-7 degrees. 
 int32_t longitude; ///< Longitude (WGS84), in 10^-7 degrees. 
 int16_t battV; ///< Battery voltage in 0.01 Volt. 
 int16_t battI; ///< Battery current in 0.01 A. 
 int16_t temp; ///< Cabin Temperature in 0.1 Celcius. 
 int16_t course; ///< Course heading in 0.01 degrees. 
 int16_t groundspeed; ///< groundspeed in 0.01 m/s. 
} mavlink_navd_t;

#define MAVLINK_MSG_ID_NavD_LEN 18
#define MAVLINK_MSG_ID_151_LEN 18

#define MAVLINK_MSG_ID_NavD_CRC 198
#define MAVLINK_MSG_ID_151_CRC 198



#define MAVLINK_MESSAGE_INFO_NavD { \
	"NavD", \
	7, \
	{  { "latitude", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_navd_t, latitude) }, \
         { "longitude", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_navd_t, longitude) }, \
         { "battV", NULL, MAVLINK_TYPE_INT16_T, 0, 8, offsetof(mavlink_navd_t, battV) }, \
         { "battI", NULL, MAVLINK_TYPE_INT16_T, 0, 10, offsetof(mavlink_navd_t, battI) }, \
         { "temp", NULL, MAVLINK_TYPE_INT16_T, 0, 12, offsetof(mavlink_navd_t, temp) }, \
         { "course", NULL, MAVLINK_TYPE_INT16_T, 0, 14, offsetof(mavlink_navd_t, course) }, \
         { "groundspeed", NULL, MAVLINK_TYPE_INT16_T, 0, 16, offsetof(mavlink_navd_t, groundspeed) }, \
         } \
}


/**
 * @brief Pack a navd message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param battV Battery voltage in 0.01 Volt. 
 * @param battI Battery current in 0.01 A. 
 * @param temp Cabin Temperature in 0.1 Celcius. 
 * @param latitude Latitude (WGS84), in 10^-7 degrees. 
 * @param longitude Longitude (WGS84), in 10^-7 degrees. 
 * @param course Course heading in 0.01 degrees. 
 * @param groundspeed groundspeed in 0.1 m/s. 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_navd_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       int16_t battV, int16_t battI, int16_t temp, int32_t latitude, int32_t longitude, int16_t course, int16_t groundspeed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_NavD_LEN];
	_mav_put_int32_t(buf, 0, latitude);
	_mav_put_int32_t(buf, 4, longitude);
	_mav_put_int16_t(buf, 8, battV);
	_mav_put_int16_t(buf, 10, battI);
	_mav_put_int16_t(buf, 12, temp);
	_mav_put_int16_t(buf, 14, course);
	_mav_put_int16_t(buf, 16, groundspeed);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_NavD_LEN);
#else
	mavlink_navd_t packet;
	packet.latitude = latitude;
	packet.longitude = longitude;
	packet.battV = battV;
	packet.battI = battI;
	packet.temp = temp;
	packet.course = course;
	packet.groundspeed = groundspeed;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_NavD_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_NavD;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_NavD_LEN, MAVLINK_MSG_ID_NavD_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_NavD_LEN);
#endif
}

/**
 * @brief Pack a navd message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param battV Battery voltage in 0.01 Volt. 
 * @param battI Battery current in 0.01 A. 
 * @param temp Cabin Temperature in 0.1 Celcius. 
 * @param latitude Latitude (WGS84), in 10^-7 degrees. 
 * @param longitude Longitude (WGS84), in 10^-7 degrees. 
 * @param course Course heading in 0.01 degrees. 
 * @param groundspeed groundspeed in 0.1 m/s. 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_navd_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           int16_t battV,int16_t battI,int16_t temp,int32_t latitude,int32_t longitude,int16_t course,int16_t groundspeed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_NavD_LEN];
	_mav_put_int32_t(buf, 0, latitude);
	_mav_put_int32_t(buf, 4, longitude);
	_mav_put_int16_t(buf, 8, battV);
	_mav_put_int16_t(buf, 10, battI);
	_mav_put_int16_t(buf, 12, temp);
	_mav_put_int16_t(buf, 14, course);
	_mav_put_int16_t(buf, 16, groundspeed);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_NavD_LEN);
#else
	mavlink_navd_t packet;
	packet.latitude = latitude;
	packet.longitude = longitude;
	packet.battV = battV;
	packet.battI = battI;
	packet.temp = temp;
	packet.course = course;
	packet.groundspeed = groundspeed;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_NavD_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_NavD;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_NavD_LEN, MAVLINK_MSG_ID_NavD_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_NavD_LEN);
#endif
}

/**
 * @brief Encode a navd struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param navd C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_navd_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_navd_t* navd)
{
	return mavlink_msg_navd_pack(system_id, component_id, msg, navd->battV, navd->battI, navd->temp, navd->latitude, navd->longitude, navd->course, navd->groundspeed);
}

/**
 * @brief Encode a navd struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param navd C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_navd_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_navd_t* navd)
{
	return mavlink_msg_navd_pack_chan(system_id, component_id, chan, msg, navd->battV, navd->battI, navd->temp, navd->latitude, navd->longitude, navd->course, navd->groundspeed);
}

/**
 * @brief Send a navd message
 * @param chan MAVLink channel to send the message
 *
 * @param battV Battery voltage in 0.01 Volt. 
 * @param battI Battery current in 0.01 A. 
 * @param temp Cabin Temperature in 0.1 Celcius. 
 * @param latitude Latitude (WGS84), in 10^-7 degrees. 
 * @param longitude Longitude (WGS84), in 10^-7 degrees. 
 * @param course Course heading in 0.01 degrees. 
 * @param groundspeed groundspeed in 0.1 m/s. 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_navd_send(mavlink_channel_t chan, int16_t battV, int16_t battI, int16_t temp, int32_t latitude, int32_t longitude, int16_t course, int16_t groundspeed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_NavD_LEN];
	_mav_put_int32_t(buf, 0, latitude);
	_mav_put_int32_t(buf, 4, longitude);
	_mav_put_int16_t(buf, 8, battV);
	_mav_put_int16_t(buf, 10, battI);
	_mav_put_int16_t(buf, 12, temp);
	_mav_put_int16_t(buf, 14, course);
	_mav_put_int16_t(buf, 16, groundspeed);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NavD, buf, MAVLINK_MSG_ID_NavD_LEN, MAVLINK_MSG_ID_NavD_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NavD, buf, MAVLINK_MSG_ID_NavD_LEN);
#endif
#else
	mavlink_navd_t packet;
	packet.latitude = latitude;
	packet.longitude = longitude;
	packet.battV = battV;
	packet.battI = battI;
	packet.temp = temp;
	packet.course = course;
	packet.groundspeed = groundspeed;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NavD, (const char *)&packet, MAVLINK_MSG_ID_NavD_LEN, MAVLINK_MSG_ID_NavD_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NavD, (const char *)&packet, MAVLINK_MSG_ID_NavD_LEN);
#endif
#endif
}

#endif

// MESSAGE NavD UNPACKING


/**
 * @brief Get field battV from navd message
 *
 * @return Battery voltage in 0.01 Volt. 
 */
static inline int16_t mavlink_msg_navd_get_battV(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  8);
}

/**
 * @brief Get field battI from navd message
 *
 * @return Battery current in 0.01 A. 
 */
static inline int16_t mavlink_msg_navd_get_battI(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  10);
}

/**
 * @brief Get field temp from navd message
 *
 * @return Cabin Temperature in 0.1 Celcius. 
 */
static inline int16_t mavlink_msg_navd_get_temp(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  12);
}

/**
 * @brief Get field latitude from navd message
 *
 * @return Latitude (WGS84), in 10^-7 degrees. 
 */
static inline int32_t mavlink_msg_navd_get_latitude(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  0);
}

/**
 * @brief Get field longitude from navd message
 *
 * @return Longitude (WGS84), in 10^-7 degrees. 
 */
static inline int32_t mavlink_msg_navd_get_longitude(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field course from navd message
 *
 * @return Course heading in 0.01 degrees. 
 */
static inline int16_t mavlink_msg_navd_get_course(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  14);
}

/**
 * @brief Get field groundspeed from navd message
 *
 * @return groundspeed in 0.1 m/s. 
 */
static inline int16_t mavlink_msg_navd_get_groundspeed(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  16);
}

/**
 * @brief Decode a navd message into a struct
 *
 * @param msg The message to decode
 * @param navd C-struct to decode the message contents into
 */
static inline void mavlink_msg_navd_decode(const mavlink_message_t* msg, mavlink_navd_t* navd)
{
#if MAVLINK_NEED_BYTE_SWAP
	navd->latitude = mavlink_msg_navd_get_latitude(msg);
	navd->longitude = mavlink_msg_navd_get_longitude(msg);
	navd->battV = mavlink_msg_navd_get_battV(msg);
	navd->battI = mavlink_msg_navd_get_battI(msg);
	navd->temp = mavlink_msg_navd_get_temp(msg);
	navd->course = mavlink_msg_navd_get_course(msg);
	navd->groundspeed = mavlink_msg_navd_get_groundspeed(msg);
#else
	memcpy(navd, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_NavD_LEN);
#endif
}
