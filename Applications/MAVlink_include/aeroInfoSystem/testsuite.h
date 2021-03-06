/** @file
 *	@brief MAVLink comm protocol testsuite generated from aeroInfoSystem.xml
 *	@see http://qgroundcontrol.org/mavlink/
 */
#ifndef AEROINFOSYSTEM_TESTSUITE_H
#define AEROINFOSYSTEM_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL

static void mavlink_test_aeroInfoSystem(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{

	mavlink_test_aeroInfoSystem(system_id, component_id, last_msg);
}
#endif




static void mavlink_test_pfd(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_pfd_t packet_in = {
		17235,
	}17339,
	}17443,
	}17547,
	}17651,
	};
	mavlink_pfd_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.roll = packet_in.roll;
        	packet1.pitch = packet_in.pitch;
        	packet1.yaw = packet_in.yaw;
        	packet1.altitude = packet_in.altitude;
        	packet1.airspeed = packet_in.airspeed;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_pfd_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_pfd_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_pfd_pack(system_id, component_id, &msg , packet1.roll , packet1.pitch , packet1.yaw , packet1.altitude , packet1.airspeed );
	mavlink_msg_pfd_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_pfd_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.roll , packet1.pitch , packet1.yaw , packet1.altitude , packet1.airspeed );
	mavlink_msg_pfd_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_pfd_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_pfd_send(MAVLINK_COMM_1 , packet1.roll , packet1.pitch , packet1.yaw , packet1.altitude , packet1.airspeed );
	mavlink_msg_pfd_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_navd(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_navd_t packet_in = {
		963497464,
	}963497672,
	}17651,
	}17755,
	}17859,
	}17963,
	}18067,
	};
	mavlink_navd_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.latitude = packet_in.latitude;
        	packet1.longitude = packet_in.longitude;
        	packet1.battV = packet_in.battV;
        	packet1.battI = packet_in.battI;
        	packet1.temp = packet_in.temp;
        	packet1.course = packet_in.course;
        	packet1.groundspeed = packet_in.groundspeed;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_navd_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_navd_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_navd_pack(system_id, component_id, &msg , packet1.battV , packet1.battI , packet1.temp , packet1.latitude , packet1.longitude , packet1.course , packet1.groundspeed );
	mavlink_msg_navd_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_navd_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.battV , packet1.battI , packet1.temp , packet1.latitude , packet1.longitude , packet1.course , packet1.groundspeed );
	mavlink_msg_navd_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_navd_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_navd_send(MAVLINK_COMM_1 , packet1.battV , packet1.battI , packet1.temp , packet1.latitude , packet1.longitude , packet1.course , packet1.groundspeed );
	mavlink_msg_navd_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_aeroInfoSystem(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_test_pfd(system_id, component_id, last_msg);
	mavlink_test_navd(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // AEROINFOSYSTEM_TESTSUITE_H
