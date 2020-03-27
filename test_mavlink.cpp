#include <time.h>
#include <mavconn/interface.h>

#include <chrono>
#include <iostream>

using namespace mavconn;
using namespace mavlink;

static void send_heartbeat(MAVConnInterface *ip) {
	using mavlink::common::MAV_TYPE;
	using mavlink::common::MAV_AUTOPILOT;
	using mavlink::common::MAV_MODE;
	using mavlink::common::MAV_STATE;

	mavlink::common::msg::HEARTBEAT hb {};
	hb.type = int(MAV_TYPE::ONBOARD_CONTROLLER);
	hb.autopilot = int(MAV_AUTOPILOT::INVALID);
	hb.base_mode = int(MAV_MODE::MANUAL_ARMED);
	hb.custom_mode = 0;
	hb.system_status = int(MAV_STATE::ACTIVE);

	ip->send_message_ignore_drop(hb);
}

static void send_sys_status(MAVConnInterface *ip) {
	mavlink_message_t msg {};
	mavlink::MsgMap map(msg);

	mavlink::common::msg::SYS_STATUS st {};
	st.load = 100;

	st.serialize(map);
	mavlink::mavlink_finalize_message(&msg, ip->get_system_id(), ip->get_component_id(), st.MIN_LENGTH, st.LENGTH, st.CRC_EXTRA);

	//const mavlink::mavlink_msg_entry_t *e = mavlink::mavlink_get_msg_entry(msg.msgid);

	ip->send_message_ignore_drop(&msg);
}

static void send_gps_global_origin(MAVConnInterface *ip) {
	mavlink_message_t msg {};
	mavlink::MsgMap map(msg);

	mavlink::common::msg::SET_GPS_GLOBAL_ORIGIN ggo {};

	int32_t lat = 425633500;   // Terni
	int32_t lon = 126432900;   // Terni
	int32_t alt = 163000;      // Terni

	ggo.latitude = lat;
	ggo.longitude = lon;	
	ggo.altitude = alt;

	ggo.serialize(map);
	mavlink::mavlink_finalize_message(&msg, ip->get_system_id(), ip->get_component_id(), ggo.MIN_LENGTH, ggo.LENGTH, ggo.CRC_EXTRA);

	ip->send_message_ignore_drop(&msg);
}

static void send_set_home_position(MAVConnInterface *ip) {
	mavlink_message_t msg {};
	mavlink::MsgMap map(msg);

	mavlink::common::msg::SET_HOME_POSITION shp {};

	int32_t lat = 425633500;   // Terni
	int32_t lon = 126432900;   // Terni
	int32_t alt = 163000;      // Terni
	//std::array<float, 4> q = {1, 0, 0, 0};   // w x y z

	shp.latitude = lat;
	shp.longitude = lon;	
	shp.altitude = alt;
	shp.x = 0.0;
	shp.y = 0.0;
	shp.z = 0.0;
	shp.q = {1.0, 0.0, 0.0, 0.0};   // w x y z
	shp.approach_x = 0.0;
	shp.approach_y = 0.0;
	shp.approach_z = 1.0;

	shp.serialize(map);
	mavlink::mavlink_finalize_message(&msg, ip->get_system_id(), ip->get_component_id(), shp.MIN_LENGTH, shp.LENGTH, shp.CRC_EXTRA);

	ip->send_message_ignore_drop(&msg);
}

static void send_vision_position_estimate(MAVConnInterface *ip) {
	mavlink_message_t msg {};
	mavlink::MsgMap map(msg);

	mavlink::common::msg::VISION_POSITION_ESTIMATE vpe {};

	vpe.x = 0.0;
	vpe.y = 0.0;
	vpe.z = 0.0;
	vpe.roll = 0.0;
	vpe.pitch = 0.0;
	vpe.yaw = 0.0;

	vpe.serialize(map);
	mavlink::mavlink_finalize_message(&msg, ip->get_system_id(), ip->get_component_id(), vpe.MIN_LENGTH, vpe.LENGTH, vpe.CRC_EXTRA);

	ip->send_message_ignore_drop(&msg);
}

int main(int argc, char **argv){
	MAVConnInterface::Ptr client;

	// create client
	//client = MAVConnInterface::open_url("/dev/ttyACM0:115200", 44, 200);
	client = MAVConnInterface::open_url("tcp://192.168.10.16:2000", 44, 200);

	client->message_received_cb = [&](const mavlink_message_t * msg, const Framing framing) {
		// std::cout << "msgid: " << int(msg->msgid) << " len: " << int(msg->len);
		// std::cout << " sysid: " << int(msg->sysid) << " compid: " << int(msg->compid);
		// std::cout << " framing: " << int(framing) << std::endl;

		int msgid = int(msg->msgid);

		if (msgid == mavlink::common::msg::STATUSTEXT::MSG_ID) {
			mavlink::common::msg::STATUSTEXT stt {};
			mavlink::MsgMap map(msg);
			
			std::cout << "msgid: " << int(msg->msgid) << " len: " << int(msg->len);
			std::cout << " sysid: " << int(msg->sysid) << " compid: " << int(msg->compid);
			std::cout << " framing: " << int(framing) << std::endl;

			stt.deserialize(map);
			std::cout << "STATUSTEXT: " << to_string(stt.text) << std::endl;
		}

	};

	using namespace std::chrono;

	u_int64_t last_hb_send = 0;
	u_int64_t last_vpe_send = 0;
    u_int64_t now_millis = 0;

	while (true) {
		now_millis = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
		
		if (now_millis - last_hb_send > 1000) {
			send_heartbeat(client.get());
			send_gps_global_origin(client.get());
			send_set_home_position(client.get());

			//std::cout << now_millis << std::endl;

			last_hb_send = now_millis;
		}

		if (now_millis - last_vpe_send > 50) {
			send_vision_position_estimate(client.get());

			//std::cout << now_millis << std::endl;

			last_vpe_send = now_millis;
		}

		//sleep(1.0);
	}
	return 0;
}
