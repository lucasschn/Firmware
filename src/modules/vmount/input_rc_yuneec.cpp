/****************************************************************************
*
*   Copyright (c) 2016-2017 PX4 Development Team. All rights reserved.
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
 * @file input_rc_yuneec.cpp
 * @author Beat Küng <beat-kueng@gmx.net>
 *
 */

#include "input_rc_yuneec.h"

#include <errno.h>
#include <px4_posix.h>
#include <px4_defines.h>
#include <mathlib/mathlib.h>


namespace vmount
{


InputRCYuneec::InputRCYuneec(float stick_deadzone)
	: InputRC(false, 0, 0, 0), _stick_deadzone(stick_deadzone)
{
}

InputRCYuneec::~InputRCYuneec()
{
}


bool InputRCYuneec::_read_control_data_from_subscription(ControlData &control_data, bool already_active)
{
	manual_control_setpoint_s manual_control_setpoint;
	orb_copy(ORB_ID(manual_control_setpoint), _get_subscription_fd(), &manual_control_setpoint);
	control_data.type = ControlData::Type::Angle;

	// take negative RC value because the gimbal output is defined inverse
	float new_aux_values[2] = {-manual_control_setpoint.aux1, -manual_control_setpoint.aux2};

	// if we were already active previously, we just update normally. Otherwise, there needs to be
	// a major stick movement to re-activate manual (or it's running for the very first time).
	bool major_movement = false;

	// Detect a big stick movement
	for (int i = 0; i < 2; ++i) {
		if (fabsf(_last_set_aux_values[i] - new_aux_values[i]) > 0.25f) {
			major_movement = true;
		}
	}

	// detect gimbal mode change
	if (_last_gimbal_yaw_mode != manual_control_setpoint.gimbal_yaw_mode ||
	    _last_gimbal_pitch_mode != manual_control_setpoint.gimbal_pitch_mode) {
		major_movement = true;
	}

	if (already_active || major_movement || _first_time) {

		_first_time = false;

		// roll
		control_data.type_data.angle.is_speed[0] = false;
		control_data.type_data.angle.angles[0] = 0.f; //not assigned
		control_data.stabilize_axis[0] = false;

		// pitch
		if (manual_control_setpoint.gimbal_pitch_mode == manual_control_setpoint_s::SWITCH_POS_ON) {
			// angular velocity pitch control
			control_data.type_data.angle.is_speed[1] = true;
			control_data.type_data.angle.angles[1] = math::deadzone(new_aux_values[1], _stick_deadzone) * M_PI_F;

		} else {
			// absolute pitch control
			// convert the input range to [-90, -3] deg
			const float pitch_min = -3.f / 180.f;
			const float pitch_max = -90.f / 180.f;
			control_data.type_data.angle.angles[1] = ((new_aux_values[1] + 1.f) / 2.f * (pitch_max - pitch_min)
					+ pitch_min) * M_PI_F;
			control_data.type_data.angle.is_speed[1] = false;
		}

		control_data.stabilize_axis[1] = true;

		// yaw
		control_data.type_data.angle.is_speed[2] = true;
		control_data.type_data.angle.angles[2] = math::deadzone(new_aux_values[0], _stick_deadzone) * M_PI_F;

		if (manual_control_setpoint.gimbal_yaw_mode == manual_control_setpoint_s::SWITCH_POS_ON) {
			// angular velocity relative to world frame
			control_data.stabilize_axis[2] = true;

		} else if (manual_control_setpoint.gimbal_yaw_mode == manual_control_setpoint_s::SWITCH_POS_MIDDLE) {
			// angular velocity relative to vehicle frame
			control_data.stabilize_axis[2] = false;

		} else if (manual_control_setpoint.gimbal_yaw_mode == manual_control_setpoint_s::SWITCH_POS_OFF) {
			// always pointing in forward vehicle direction
			control_data.stabilize_axis[2] = false;
			control_data.type_data.angle.angles[2] = 0.f;
			control_data.type_data.angle.is_speed[2] = false;
		}

		control_data.gimbal_shutter_retract = false;

		for (int i = 0; i < 2; ++i) {
			_last_set_aux_values[i] = new_aux_values[i];
		}

		_last_gimbal_pitch_mode = manual_control_setpoint.gimbal_pitch_mode;
		_last_gimbal_yaw_mode = manual_control_setpoint.gimbal_yaw_mode;

		return true;

	} else {
		return false;
	}
}


} /* namespace vmount */
