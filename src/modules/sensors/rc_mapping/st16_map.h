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
 * @file st16_map.h
 */

namespace sensors
{
namespace RCmapping
{
namespace ST16
{

// M4 version
const int M4_RAW_CHANNEL_MAPPING_VER = 0xA;
const int MAX_VALUE = 4000; // 12 bits - offset due to calibration
const int MIN_VALUE = 95; // 0 + offset due to calibration
const int KILL_HOTKEY_TIME_US = 1000000; // 1s time for kill-switch criteria
const int KILL_SWITCH_TRIGGER_COUNT = 3; // three times needs the kill switch to be high to be considered complete

// Channels
const int CHANNEL_LEFT_STICK_UP = (1 - 1);
const int CHANNEL_LEFT_STICK_RIGHT(2 - 1);
const int CHANNEL_RIGHT_STICK_UP = (3 - 1);
const int CHANNEL_RIGHT_STICK_RIGHT = (4 - 1);
const int CHANNEL_PAN_KNOB = (5 - 1);
const int CHANNEL_TILT_SLIDER = (6 - 1);
const int CHANNEL_TORTOISE_SLIDER = (7 - 1);
const int CHANNEL_THREE_WAY_SWITCH = (8 - 1);
const int CHANNEL_TWO_WAY_SWITCH = (9 - 1);
const int CHANNEL_TRIM = (10 - 1);

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
} //namespace ST16
} //namespace RCmapping
} // namespace sensors