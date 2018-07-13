
/****************************************************************************
 *
 *   Copyright (C) 2018 PX4 Development Team. All rights reserved.
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
 * @file rc_mapping.cpp
 */

#include "rc_mapping.h"
#include "st16_map.h"
#include <mathlib/mathlib.h>
#include <drivers/drv_hrt.h>

namespace sensors
{
namespace RCmapping
{
int st16_map(manual_control_setpoint_s &man, const input_rc_s &input_rc, const Parameters &parameters)
{
	// check for the M4 raw output channel mapping version embedded in channel 9 to
	// make sure change is compatible
	int version = input_rc.values[9 - 1] >> 8 & 0xF;

	if (version != ST16::M4_RAW_CHANNEL_MAPPING_VER) {
		return (int)Error::Version;
	}

	// thrust map to range [0,1]
	man.z = math::gradual((float)input_rc.values[ST16::CHANNEL_LEFT_STICK_UP],
			      0.0f, (float)ST16::MAX_VALUE,
			      0.0f, 1.0f);

	// map all other none-switch/trim channels to range [-1,1]
	auto unit_range = [](uint16_t value) {
		return math::gradual((float)value,
				     (float)ST16::MIN_VALUE, (float)ST16::MAX_VALUE,
				     -1.0f, 1.0f);
	};

	man.r = unit_range(input_rc.values[ST16::CHANNEL_LEFT_STICK_RIGHT]); // yaw
	man.x = unit_range(input_rc.values[ST16::CHANNEL_RIGHT_STICK_UP]); // pitch
	man.y = unit_range(input_rc.values[ST16::CHANNEL_RIGHT_STICK_RIGHT]); // roll
	man.aux1 = unit_range(input_rc.values[ST16::CHANNEL_PAN_KNOB]); // pan-knob / gimbal yaw
	man.aux2 = unit_range(input_rc.values[ST16::CHANNEL_TILT_SLIDER]); // tilt-slider / gimbal tilt
	man.aux5 = unit_range(input_rc.values[ST16::CHANNEL_TORTOISE_SLIDER]); // turtle-mode

	// apply rc-mode
	math::convertRcMode(parameters.rc_mode, man.y, man.x, man.r, man.z);

	// three way switches [0,1,2] -> [3,2,1]
	auto three_way = [&input_rc](ST16::ThreeWay sw) {
		return (3 - ((input_rc.values[ST16::CHANNEL_THREE_WAY_SWITCH] >> (int)sw * 2) & 0x3));
	};

	// Mode slots define the mode of the vehicle.
	// - up (=OFF) is reserved for Altitude
	// - middle is reserved for Position
	// - down (=ON) is reserved for Return
	//
	int mode_switch = three_way(ST16::ThreeWay::mode_switch);

	if (mode_switch == manual_control_setpoint_s::SWITCH_POS_OFF) {
		// switch is up
		man.mode_slot = manual_control_setpoint_s::MODE_SLOT_1;

	} else if (mode_switch == manual_control_setpoint_s::SWITCH_POS_MIDDLE) {
		// switch is middle
		man.mode_slot = manual_control_setpoint_s::MODE_SLOT_2;

	} else if (mode_switch == manual_control_setpoint_s::SWITCH_POS_ON) {
		//switch is down
		man.mode_slot = manual_control_setpoint_s::MODE_SLOT_3;
	}

	// mode switch needs to be set to None such that slots are considered
	man.mode_switch = manual_control_setpoint_s::SWITCH_POS_NONE;
	// remaining three way switches
	man.obsavoid_switch = three_way(ST16::ThreeWay::obs_avoid_switch);
	man.gimbal_yaw_mode = three_way(ST16::ThreeWay::pan_switch);
	man.gimbal_tilt_mode = three_way(ST16::ThreeWay::tilt_switch);

	// two way switches [1,0] -> [1,3]
	auto two_way = [&input_rc](ST16::TwoWay sw, const int channel) {

		if ((input_rc.values[channel] >> (int)sw) & 0x1) {
			return manual_control_setpoint_s::SWITCH_POS_ON;

		} else {
			return manual_control_setpoint_s::SWITCH_POS_OFF;
		}
	};
	man.gear_switch = two_way(ST16::TwoWay::gear_switch, ST16::CHANNEL_TWO_WAY_SWITCH);
	man.arm_switch = two_way(ST16::TwoWay::arm_button, ST16::CHANNEL_TWO_WAY_SWITCH);

	// Kill hotkey: pressing the arm button three times with low throttle within KILL_HOTKEY_TIME_US
	// will trigger kill-switch.
	static bool kill_state = false; // the kill state in which we lockdown the motors until restart
	static hrt_abstime kill_hotkey_start_time = 0; // the time when the hotkey started to measure timeout
	static int kill_hotkey_count = 0; //  how many times the button was pressed during the hotkey timeout
	static bool arm_button_pressed_last = false; //if the button was pressed last time to detect a transition

	const bool first_time = kill_hotkey_count == 0;
	const bool hotkey_complete = kill_hotkey_count >= ST16::KILL_SWITCH_TRIGGER_COUNT;
	const bool within_timeout = hrt_elapsed_time(&kill_hotkey_start_time) < ST16::KILL_HOTKEY_TIME_US;

	if (hotkey_complete) {
		kill_state = true;
	}

	if (man.z < 0.25f && (first_time || within_timeout) && !hotkey_complete) {
		if (!arm_button_pressed_last && man.arm_switch == manual_control_setpoint_s::SWITCH_POS_ON) {
			if (first_time) {
				kill_hotkey_start_time = hrt_absolute_time();
			}

			kill_hotkey_count++;
		}

	}  else {
		// reset
		kill_hotkey_count = 0;
		kill_hotkey_start_time = 0;
	}

	arm_button_pressed_last = man.arm_switch == manual_control_setpoint_s::SWITCH_POS_ON;
	man.kill_switch = kill_state;

	//////// TODO: add remaining buttons
	//=> man.aux_button = two_way(ST16::TwoWay::gear_switch, ST16::CHANNEL_TWO_WAY_SWITCH);
	//man.photo_button
	//man.video_button
	// all trim buttons...
	///////////////////////////////////////////////////////////////////////////////////////////////

	man.timestamp = input_rc.timestamp_last_signal;


	// no errors
	return (int)Error::None;
}
} // namespace RCmapping
} // namespace sensors
