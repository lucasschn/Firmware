/****************************************************************************
 *
 *   Copyright (c) 2019 YUNEEC. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file yuneec_version_query.h
 *
 */

#pragma once

#include "subscriber_handler.h"

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_command.h>

#define H520_SYSTEM_ID 1
#define H520_AUTOPILOT_COMPONENT_ID 1
#define H520_CAMERA_COMPONENT_ID 100
#define H520_GIMBAL_COMPONENT_ID 154


namespace events
{
namespace yuneec_version_query
{




class VersionQuery
{
public:

	VersionQuery(const events::SubscriberHandler &subscriber_handler);

	/** regularily called to handle state updates */
	void process();

private:
	/**
	 * check for topic updates
	 * @return true if one or more topics got updated
	 */
	bool check_for_updates();

	/** Send a vehicle_command */
	inline void send_vehicle_command(const vehicle_command_s &cmd);

	const events::SubscriberHandler &_subscriber_handler;

	orb_advert_t _vehicle_command_pub = nullptr;
	struct vehicle_status_s	_vehicle_status = {};

	unsigned char _loop_counter = 0;

};

} /* namespace yuneec_version_query */
} /* namespace events */
