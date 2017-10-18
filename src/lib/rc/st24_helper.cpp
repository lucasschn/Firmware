/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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

#include "st24_helper.h"
#include "st24.h"

#include <px4_log.h>
#include <drivers/drv_hrt.h>

void St24Helper::init(int rcs_fd)
{
	_vehicle_landed_sub = orb_subscribe(ORB_ID(vehicle_land_detected));
	_vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status_flags));
	_rcs_fd = rcs_fd;
}

void St24Helper::bind()
{
	_binding_state = BindingState::START;
	dsm_config(_rcs_fd);
#ifdef RF_RADIO_POWER_CONTROL
	RF_RADIO_POWER_CONTROL(false);
	usleep(1000);
	RF_RADIO_POWER_CONTROL(true);
	usleep(10000);
#endif

	ReceiverFcPacket *bind_packet = st24_get_bind_packet();
	// send 3 more bytes 2 for header and 1 for crc
	int ret = ::write(_rcs_fd, (uint8_t *)bind_packet, (bind_packet->length + 3));
	PX4_INFO("Sent ST24 bind command res: %d", ret);
}

void St24Helper::cycle(bool rc_lost, bool armed)
{
	orb_check(_vehicle_landed_sub, &_vehicle_landed_updated);

	if (armed) { // do nothing while armed
		return;
	}

	if (_vehicle_landed_updated) {
		vehicle_land_detected_s vehicle_landed_state;
		orb_copy(ORB_ID(vehicle_land_detected), _vehicle_landed_sub, &vehicle_landed_state);

		vehicle_status_flags_s v_flags;
		orb_copy(ORB_ID(vehicle_status_flags), _vehicle_status_sub, &v_flags);

		// Bind the aircraft only if it is upsidedown and it does not have a valid RC binding
		if (!_was_inverted && vehicle_landed_state.inverted && rc_lost &&
		    !(v_flags.conditions & vehicle_status_flags_s::CONDITION_CALIBRATION_ENABLE_MASK)) {
			bind();
		}

		_was_inverted = vehicle_landed_state.inverted;
	}

	// RC is bound to the aircraft
	if (!rc_lost && _binding_state == BindingState::WAIT) {
		_binding_state = BindingState::END;
	}

	// send led messages only in start or in end state
	if (_binding_state == BindingState::START || _binding_state == BindingState::END) {
		led_control_s leds;
		leds.timestamp = hrt_absolute_time();

		// binding led message
		if (_binding_state == BindingState::START) {
			leds.led_mask = 0xFF;
			leds.color = led_control_s::COLOR_YELLOW;
			leds.mode = led_control_s::MODE_BLINK_FAST;
			leds.num_blinks = 0;
			leds.priority = 2;
			_binding_state = BindingState::WAIT;
		}

		// stop leds
		if (_binding_state == BindingState::END) {
			leds.led_mask = 0xFF;
			leds.mode = led_control_s::MODE_DISABLED;
			leds.priority = 2;
			_binding_state = BindingState::BOUND;
		}

		// publish led msg
		if (_led_control_pub != nullptr) {
			orb_publish(ORB_ID(led_control), _led_control_pub, &leds);

		} else {
			_led_control_pub = orb_advertise(ORB_ID(led_control), &leds);
		}
	}
}
