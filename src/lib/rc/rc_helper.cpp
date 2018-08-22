/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

#ifdef ENABLE_RC_HELPER

#include "rc_helper.h"

#include <drivers/drv_hrt.h>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_status_flags.h>

void RCHelper::init()
{
	_rc_sub = orb_subscribe(ORB_ID(input_rc));
	_landed_sub = orb_subscribe(ORB_ID(vehicle_land_detected));
	_status_sub = orb_subscribe(ORB_ID(vehicle_status_flags));
}

void RCHelper::pair()
{
	_state = START;

	/* send command to ofdm module via command API */
	struct vehicle_command_s vcmd = {
		.timestamp = hrt_absolute_time(),
		.param5 = 0.0,
		.param6 = 0.0,
		.param1 = 0.0,
		.param2 = 0.0,
		.param3 = 0.0,
		.param4 = 0.0,
		.param7 = 0.0,
		.command = vehicle_command_s::VEHICLE_CMD_START_RX_PAIR,
		/* send this command to itself */
		.target_system = 1,
		.target_component = 1,
		.source_system = 1,
		.source_component = 1,
		.confirmation = false,
		.from_external = false
	};

	if (_vehicle_cmd_pub != nullptr) {
		orb_publish(ORB_ID(vehicle_command), _vehicle_cmd_pub, &vcmd);

	} else {
		_vehicle_cmd_pub = orb_advertise(ORB_ID(vehicle_command), &vcmd);

	}
}

void RCHelper::cycle(bool is_armed)
{
	input_rc_s rc = {};
	vehicle_land_detected_s landed = {};
	vehicle_status_flags_s status = {};
	bool updated = false;
	bool status_updated = false;

	/* send the command not more than once in a running cycle and do nothing if armed */
	if (_state == PAIRED || is_armed) {
		return;
	}

	/* check whether rc has been lost, rc input values are typically from a MAVLink msg */
	orb_check(_rc_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(input_rc), _rc_sub, &rc);
		_rc_last_signal_ts = rc.timestamp_last_signal;

	} else if ((hrt_absolute_time() - _rc_last_signal_ts) > 1000 * 1000) {
		rc.rc_lost = true;
	}

	updated = false;
	orb_check(_landed_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_land_detected), _landed_sub, &landed);
	}

	orb_check(_status_sub, &status_updated);

	if (status_updated) {
		orb_copy(ORB_ID(vehicle_status_flags), _status_sub, &status);
	}

	if (updated || status_updated) {
		/* pair only if it's upsidedown and rc has been lost */
		if (!_was_inverted && landed.inverted && rc.rc_lost &&
		    !status.condition_calibration_enabled) {
			pair();
		}

		_was_inverted = landed.inverted;
	}

	if (!rc.rc_lost) {
		_state = PREPAIRED;
	}

	if (_state == START || _state == PREPAIRED) {
		led_control_s leds = {};
		leds.led_mask = 0xff;
		leds.priority = 2;
		leds.timestamp = hrt_absolute_time();

		/* pack a led message */
		if (_state == START) {
			leds.color = led_control_s::COLOR_YELLOW;
			leds.mode = led_control_s::MODE_BLINK_FAST;
			leds.num_blinks = 0;
			_state = PAIRING;

		} else if (_state == PREPAIRED) {
			leds.mode = led_control_s::MODE_DISABLED;
			_state = PAIRED;
		}

		/* publish the led message */
		if (_led_control_pub != nullptr) {
			orb_publish(ORB_ID(led_control), _led_control_pub, &leds);

		} else {
			_led_control_pub = orb_advertise(ORB_ID(led_control), &leds);
		}
	}
}

#endif // ENABLE_RC_HELPER
