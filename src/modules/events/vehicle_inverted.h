/****************************************************************************
 *
 *   Copyright (c) 2018 ATL | Yuneec Research. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file vehicle_inverted.h
 * Detect vehicle inverted state to set the landing gear accordig to the RC switch
 *
 * @author Simone Guscetti <simone@yuneecresearch.com>
 */

#pragma once

#include "subscriber_handler.h"

#include <drivers/drv_hrt.h>

#include <uORB/uORB.h>
#include <uORB/topics/landing_gear.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_land_detected.h>

namespace events
{
namespace inverted
{

class VehicleInverted
{
public:
	VehicleInverted(const events::SubscriberHandler &subscriber_handler);

	/** regularily called to handle state updates */
	void process();

protected:
	/**
	 * check for topic updates
	 * @return true if one or more topics got updated
	 */
	bool check_for_updates();

	/** publish landing gear state */
	void publish();

	struct manual_control_setpoint_s _manual_control_sp {};
	struct vehicle_land_detected_s _land_detector {};
	struct vehicle_status_flags_s _status_flags {};

	struct landing_gear_s _landing_gear {};
private:
	bool _pwm_armed = false;
	uint8_t _gear_pos_prev = manual_control_setpoint_s::SWITCH_POS_NONE;
	orb_advert_t _landing_gear_pub{nullptr};
	const events::SubscriberHandler &_subscriber_handler;

	int arm_pwm(bool arm);
};

} /* namespace inverted */
} /* namespace events */
