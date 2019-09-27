/****************************************************************************
 *
 *   Copyright (c) 2019 YUNEEC. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file yuneec_flexigraph.cpp
 *
 */

#include "yuneec_flexigraph.h"

#include <v2.0/yuneec/mavlink.h>

#include <px4_defines.h>
#include <stdint.h>


namespace events
{
namespace yuneec_flexigraph
{

YuneecFlexigraph::YuneecFlexigraph(const events::SubscriberHandler &subscriber_handler)
	: _subscriber_handler(subscriber_handler)
{
}

bool YuneecFlexigraph::check_for_updates()
{
	if (_subscriber_handler.manual_control_sp_updated()) {
		orb_copy(ORB_ID(manual_control_setpoint), _subscriber_handler.get_manual_control_setpoint_sub(),
			 &_manual_control_setpoint);
		return true;
	}

	return false;
}

void YuneecFlexigraph::process()
{
	if (!check_for_updates()) {
		return;
	}


	// Check if payload-release button was pushed
	if (_manual_control_setpoint.flexi_release == manual_control_setpoint_s::SWITCH_POS_ON) {
		request_payload_drop();
	}

}

void YuneecFlexigraph::request_payload_drop()
{
	// Send vehicle command message to drop the payload
	_cmd = vehicle_command_s{};
	_cmd.command = vehicle_command_s::VEHICLE_CMD_IMAGE_START_CAPTURE;
	_cmd.param1 = 0; // reserved
	_cmd.param2 = 0; // Interval time [seconds]
	_cmd.param3 = 1; // Count
	_cmd.target_system = 1;
	_cmd.target_component = MAV_COMP_ID_PERIPHERAL;

	if (_vehicle_command_pub != nullptr) {
		orb_publish(ORB_ID(vehicle_command), _vehicle_command_pub, &_cmd);

	} else	{
		_vehicle_command_pub = orb_advertise_queue(ORB_ID(vehicle_command), &_cmd, vehicle_command_s::ORB_QUEUE_LENGTH);
	}
}

} /* namespace yuneec_flexigraph */
} /* namespace events */
