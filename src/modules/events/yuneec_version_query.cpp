/****************************************************************************
 *
 *   Copyright (c) 2019 YUNEEC. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file yuneec_version_query.cpp
 *
 */

#include "yuneec_version_query.h"

#include <px4_defines.h>
#include <px4_log.h>

#include <matrix/math.hpp>


namespace events
{
namespace yuneec_version_query
{

VersionQuery::VersionQuery(const events::SubscriberHandler &subscriber_handler)
	: _subscriber_handler(subscriber_handler)
{
}

bool VersionQuery::check_for_updates()
{
	bool ret = false;

	if (_subscriber_handler.vehicle_status_updated()) {
		orb_copy(ORB_ID(vehicle_status), _subscriber_handler.get_vehicle_status_sub(), &_vehicle_status);
		ret = true;
	}

	return ret;
}

void VersionQuery::send_vehicle_command(const vehicle_command_s &cmd)
{
	if (_vehicle_command_pub != nullptr) {
		orb_publish(ORB_ID(vehicle_command), _vehicle_command_pub, &cmd);

	} else	{
		_vehicle_command_pub = orb_advertise_queue(ORB_ID(vehicle_command), &cmd, vehicle_command_s::ORB_QUEUE_LENGTH);
	}
}

void VersionQuery::process()
{
	check_for_updates();

	// Query versions every 5 seconds (given 30Hz loop execution)
	if (_vehicle_status.arming_state != vehicle_status_s::ARMING_STATE_ARMED && _loop_counter++ == 150) {
		_loop_counter = 0;

		// Request gimbal version
		{
			vehicle_command_s cmd{};
			cmd.command = vehicle_command_s::VEHICLE_CMD_REQUEST_AUTOPILOT_CAPABILITIES;
			cmd.target_system = H520_SYSTEM_ID;
			cmd.target_component = H520_GIMBAL_COMPONENT_ID;
			cmd.source_system = H520_SYSTEM_ID;
			cmd.source_component = H520_AUTOPILOT_COMPONENT_ID;
			cmd.param1 = 1; // Request firmware version
			cmd.from_external = false;
			send_vehicle_command(cmd);
		}

		// Request camera version
		{
			vehicle_command_s cmd{};
			cmd.command = vehicle_command_s::VEHICLE_CMD_REQUEST_CAMERA_INFORMATION;
			cmd.target_system = H520_SYSTEM_ID;
			cmd.target_component = H520_CAMERA_COMPONENT_ID;
			cmd.source_system = H520_SYSTEM_ID;
			cmd.source_component = H520_AUTOPILOT_COMPONENT_ID;
			cmd.param1 = 1; // Request firmware version
			cmd.from_external = false;
			send_vehicle_command(cmd);
		}
	}
}

} /* namespace yuneec_version_query */
} /* namespace events */
