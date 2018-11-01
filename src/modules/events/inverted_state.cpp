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
 * @file inverted_state.cpp
 * Inverted State decouples part of the landing gear logic form the position controller
 *
 * @author Simone Guscetti <simone@yuneecresearch.com>
 *
 */

#include "inverted_state.h"

#include <px4_log.h>

namespace events
{
namespace states
{

InvertedState::InvertedState(const events::SubscriberHandler &subscriber_handler)
	: _subscriber_handler(subscriber_handler) {}

bool InvertedState::check_for_updates()
{
	bool got_updates = false;

	if (_subscriber_handler.manual_control_sp_updated()) {
		orb_copy(ORB_ID(manual_control_setpoint), _subscriber_handler.get_manual_control_sp_sub(), &_manual_control_sp);
		got_updates = true;
	}

	if (_subscriber_handler.vehicle_land_detcted_updated()) {
		orb_copy(ORB_ID(vehicle_land_detected), _subscriber_handler.get_vehicle_land_detected_sub(), &_land_detector);
		got_updates = true;
	}

	return got_updates;
}

void InvertedState::process()
{
	if (!check_for_updates()) {
		return;
	}

	if (_land_detector.inverted) {
		// TODO set PWM output to arm -> remove diff from FMU
		if (_gear_pos_prev != _manual_control_sp.gear_switch) {
			_landing_gear.landing_gear =
				(_manual_control_sp.gear_switch == manual_control_setpoint_s::SWITCH_POS_ON) ?
				landing_gear_s::LANDING_GEAR_UP :
				landing_gear_s::LANDING_GEAR_DOWN;

			publish();
		}
	}

	_gear_pos_prev = _manual_control_sp.gear_switch;
}

void InvertedState::publish()
{
	_landing_gear.timestamp = hrt_absolute_time();

	if (_landing_gear_pub != nullptr) {
		orb_publish(ORB_ID(landing_gear), _landing_gear_pub, &_landing_gear);

	} else {
		_landing_gear_pub =  orb_advertise(ORB_ID(landing_gear), &_landing_gear);
	}
}

} /* namespace gear */
} /* namespace events */
