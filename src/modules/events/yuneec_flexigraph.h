/****************************************************************************
 *
 *   Copyright (c) 2019 YUNEEC. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file yuneec_flexigraph.h
 *
 */

#pragma once

#include "subscriber_handler.h"

#include <uORB/uORB.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_command.h>

namespace events
{
namespace yuneec_flexigraph
{

class YuneecFlexigraph
{
public:

	YuneecFlexigraph(const events::SubscriberHandler &subscriber_handler);

	/** regularily called to handle state updates */
	void process();

private:
	/**
	 * check for topic updates
	 * @return true if one or more topics got updated
	 */
	bool check_for_updates();

	/** Publish mavlink message to request flexigraph to drop its payload */
	void request_payload_drop();

	struct vehicle_command_s _cmd = {};
	struct manual_control_setpoint_s _manual_control_setpoint = {};
	const events::SubscriberHandler &_subscriber_handler;
	orb_advert_t _vehicle_command_pub = nullptr;
};

} /* namespace yuneec_flexigraph */
} /* namespace events */
