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
 * @file ST16.hpp
 *
 * Defining ST16 channel parsing to map input data to manual_control_setpoint.
 *
 * @author Dennis Mannhart <dennis@yuneecresearch.com>
 * @author Matthias Grob <maetugr@gmail.com>
 */

#pragma once

#include "RCMap.hpp"

#include <mathlib/mathlib.h>
#include <drivers/drv_hrt.h>

namespace sensors
{

class RCMapST16 : public RCMap
{
public:
	RCMapST16() = default;
	virtual ~RCMapST16() = default;

	static constexpr int RAW_CHANNEL_MAPPING_VER_ST16 = 0xA; // Remote type and mapping version

	/**
	 * Map ST16 data (channels) to the manual_control_setpoint
	 * @return error code according to RCMap::Error
	 */
	virtual int map(manual_control_setpoint_s &man, const input_rc_s &input_rc, const sensors::Parameters &parameters)
	{
		// map all none-switch/trim channels to range [-1,1]
		man.z = unit_range(input_rc.values[CHANNEL_LEFT_STICK_UP]); // thrust
		man.r = unit_range(input_rc.values[CHANNEL_LEFT_STICK_RIGHT]); // yaw
		man.x = unit_range(input_rc.values[CHANNEL_RIGHT_STICK_UP]); // pitch
		man.y = unit_range(input_rc.values[CHANNEL_RIGHT_STICK_RIGHT]); // roll
		man.aux1 = unit_range(input_rc.values[CHANNEL_PAN_KNOB]); // pan-knob / gimbal yaw
		man.aux2 = unit_range(input_rc.values[CHANNEL_TILT_SLIDER]); // tilt-slider / gimbal tilt
		man.aux5 = unit_range(input_rc.values[CHANNEL_TORTOISE_SLIDER]); // turtle-mode

		// apply rc-mode (-> swap sticks)
		math::convertRcMode(parameters.rc_mode, man.y, man.x, man.r, man.z);

		// re-map throttle from [-1,1] -> [0,1]
		man.z = math::gradual(man.z, -1.0f, 1.0f, 0.0f, 1.0f);

		// Mode slots define the mode of the vehicle.
		int mode_switch = three_way(ThreeWay::mode_switch, input_rc);

		switch (mode_switch) {
		case manual_control_setpoint_s::SWITCH_POS_OFF: // switch is up - Altitude
			man.mode_slot = manual_control_setpoint_s::MODE_SLOT_1;
			break;

		case manual_control_setpoint_s::SWITCH_POS_MIDDLE: // switch is middle - Position
			man.mode_slot = manual_control_setpoint_s::MODE_SLOT_2;
			break;

		case manual_control_setpoint_s::SWITCH_POS_ON: // switch is down - Return
			man.mode_slot = manual_control_setpoint_s::MODE_SLOT_3;
			break;

		default:
			man.mode_slot = manual_control_setpoint_s::MODE_SLOT_NONE;
		}

		// mode switch needs to be set to None such that slots are considered
		man.mode_switch = manual_control_setpoint_s::SWITCH_POS_NONE;

		// remaining switches
		man.obsavoid_switch = three_way(ThreeWay::obs_avoid_switch, input_rc);
		man.gimbal_yaw_mode = three_way(ThreeWay::pan_switch, input_rc);
		man.gimbal_pitch_mode = three_way(ThreeWay::tilt_switch, input_rc);
		man.gear_switch = two_way(TwoWay::gear_switch, CHANNEL_TWO_WAY_SWITCH, input_rc);
		man.arm_switch = two_way(TwoWay::arm_button, CHANNEL_TWO_WAY_SWITCH, input_rc);

		updateKillSwitch(man);

		// TODO: add remaining buttons: AUX, photo, video, trim

		man.timestamp = input_rc.timestamp_last_signal;
		return (int)RCMap::Error::None;
	}

	/**
	 * Map ST16 team mode slave data (channels) on top of the existing manual_control_setpoint
	 * @return error code according to RCMap::Error
	 */
	int mapSlave(manual_control_setpoint_s &man, const input_rc_s &input_rc, const sensors::Parameters &parameters)
	{

		// check for the M4 raw output channel mapping version embedded in channel 9 to
		// make sure change is compatible
		int version = input_rc.values[9 - 1] >> 8 & 0xF;

		if (version != RAW_CHANNEL_MAPPING_VER_ST16) {
			return (int)Error::Version;
		}

		man.gimbal_yaw_mode = three_way(ThreeWay::pan_switch, input_rc); // OFF: heading 0, MNIDDLE: angle, ON: angle stabilized
		man.gimbal_pitch_mode = three_way(ThreeWay::tilt_switch, input_rc); // OFF/MIDDLE: angle, ON: velocity

		man.aux1 = unit_range(input_rc.values[CHANNEL_RIGHT_STICK_RIGHT]); // camera pan (= yaw)

		// camera tilt (= pitch)
		if (man.gimbal_pitch_mode == manual_control_setpoint_s::SWITCH_POS_ON) {
			man.aux2 = unit_range(input_rc.values[CHANNEL_LEFT_STICK_UP]); // tilt speed control: use stick inputs

		} else {
			man.aux2 = unit_range(input_rc.values[CHANNEL_TILT_SLIDER]); // tilt angle control: use tilt slider

		}

		// do not overwrite man.timestamp because master is the main required input that triggers rc loss!
		return (int)RCMap::Error::None;
	}

private:
	static constexpr int MAX_VALUE = 4000; // 12 bits - offset due to calibration
	static constexpr int MIN_VALUE = 95; // 0 + offset due to calibration
	static constexpr int KILL_HOTKEY_TIME_US = 1000000; // 1s time for kill-switch criteria
	static constexpr int KILL_SWITCH_TRIGGER_COUNT =
		3; // three times needs the kill switch to be high to be considered complete

	// Channels
	static constexpr int CHANNEL_LEFT_STICK_UP = (1 - 1);
	static constexpr int CHANNEL_LEFT_STICK_RIGHT = (2 - 1);
	static constexpr int CHANNEL_RIGHT_STICK_UP = (3 - 1);
	static constexpr int CHANNEL_RIGHT_STICK_RIGHT = (4 - 1);
	static constexpr int CHANNEL_PAN_KNOB = (5 - 1);
	static constexpr int CHANNEL_TILT_SLIDER = (6 - 1);
	static constexpr int CHANNEL_TORTOISE_SLIDER = (7 - 1);
	static constexpr int CHANNEL_THREE_WAY_SWITCH = (8 - 1);
	static constexpr int CHANNEL_TWO_WAY_SWITCH = (9 - 1);
	static constexpr int CHANNEL_TRIM = (10 - 1);

	enum class ThreeWay : int {
		mode_switch = 0,
		obs_avoid_switch,
		pan_switch,
		tilt_switch
	};

	enum class TwoWay : int {
		gear_switch = 0,
		arm_button,
		aux_button,
		photo_button,
		video_button,
	};

	enum class Trim : int {
		left_trim_up = 0,
		left_trim_down,
		left_trim_left,
		left_trim_right,
		right_trim_up,
		right_trim_down,
		right_trim_left,
		right_trim_right
	};

	/**
	 * Convert 12-Bit M4 channel values to unit floats
	 * but cutting of the margin a bit (95) to make sure the extreme values are reachable
	 * @param value [0,4095] channel value
	 * @return [-1,1] floats
	 */
	float unit_range(uint16_t value)
	{
		return math::gradual<float>(value, MIN_VALUE, MAX_VALUE, -1.0f, 1.0f);
	};

	/**
	 * Convert three way switches
	 * from their channel bits [0,1,2] to manual_control_setpoint SWITCH_POS
	 * @param sw bit offset of the switch inside the channel data
	 * @param channel RC channel number containing the information
	 * @param input_rc RC channel data
	 * @return corresponding manual_control_setpoint_s::SWITCH_POS
	 */
	int three_way(ThreeWay sw, const input_rc_s &input_rc)
	{
		return (3 - ((input_rc.values[CHANNEL_THREE_WAY_SWITCH] >> (int)sw * 2) & 0x3));
	};

	/**
	 * Convert two way switches
	 * from their channel bits [0,1] to manual_control_setpoint SWITCH_POS
	 * @param sw bit offset of the switch inside the channel data [0,1]
	 * @param channel RC channel number containing the information
	 * @param input_rc RC channel data
	 * @return corresponding manual_control_setpoint_s::SWITCH_POS
	 */
	int two_way(TwoWay sw, const int channel, const input_rc_s &input_rc)
	{

		if ((input_rc.values[channel] >> (int)sw) & 0x1) {
			return manual_control_setpoint_s::SWITCH_POS_ON;

		} else {
			return manual_control_setpoint_s::SWITCH_POS_OFF;
		}
	};

	// Kill switch logic states
	bool _kill_state = false; // the kill state in which we lockdown the motors until restart
	hrt_abstime _kill_hotkey_start_time = 0; // the time when the hotkey started to measure timeout
	int _kill_hotkey_count = 0; //  how many times the button was pressed during the hotkey timeout
	bool _arm_button_pressed_last = false; //if the button was pressed last time to detect a transition

	/**
	 * Process Kill switch shortcut
	 * to trigger: press the arm button three times with low throttle within KILL_HOTKEY_TIME_US
	 * @param sw bit offset of the switch inside the channel data [0,1]
	 * @param channel RC channel number containing the information
	 * @param input_rc RC channel data
	 * @return corresponding manual_control_setpoint_s::SWITCH_POS
	 */
	void updateKillSwitch(manual_control_setpoint_s &man)
	{
		const bool first_time = _kill_hotkey_count == 0;
		const bool hotkey_complete = _kill_hotkey_count >= KILL_SWITCH_TRIGGER_COUNT;
		const bool within_timeout = hrt_elapsed_time(&_kill_hotkey_start_time) < KILL_HOTKEY_TIME_US;

		if (hotkey_complete) {
			_kill_state = true;
		}

		if (man.z < 0.25f && (first_time || within_timeout) && !hotkey_complete) {
			if (!_arm_button_pressed_last && man.arm_switch == manual_control_setpoint_s::SWITCH_POS_ON) {
				if (first_time) {
					_kill_hotkey_start_time = hrt_absolute_time();
				}

				_kill_hotkey_count++;
			}

		}  else {
			// reset
			_kill_hotkey_count = 0;
			_kill_hotkey_start_time = 0;
		}

		_arm_button_pressed_last = man.arm_switch == manual_control_setpoint_s::SWITCH_POS_ON;
		man.kill_switch = _kill_state;
	}
};

} // namespace sensors
