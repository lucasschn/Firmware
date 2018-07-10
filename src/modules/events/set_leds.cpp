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

/**
 * @file set_leds.cpp
 * Separate the set_leds() function
 *
 * @author Simone Guscetti <simone@px4.io>
 * @author Beat Küng <beat-kueng@gmx.net>
 * @editor Christoph Tobler <christoph@px4.io>
 */

#include "status_display.h"

// #include <system_config.h>
#include <board_config.h>
#include <px4_log.h>
#include <matrix/math.hpp>
#include <drivers/drv_led.h>

#ifndef BOARD_FRONT_LED_MASK
#define BOARD_FRONT_LED_MASK (0)
#endif

#ifndef BOARD_REAR_LED_MASK
#define BOARD_REAR_LED_MASK  (0)
#endif

#ifndef BOARD_LEFT_LED_MASK
#define BOARD_LEFT_LED_MASK  (0)
#endif

#ifndef BOARD_RIGHT_LED_MASK
#define BOARD_RIGHT_LED_MASK (0)
#endif

namespace events
{
namespace status
{

void StatusDisplay::set_leds()
{
	bool gps_lock_valid = _vehicle_status_flags.condition_global_position_valid;
	bool home_position_valid = _vehicle_status_flags.condition_home_position_valid;
	int nav_state = _vehicle_status.nav_state;
	bool restore_position_lights = _old_nav_state == vehicle_status_s::NAVIGATION_STATE_SMART
				       && nav_state != vehicle_status_s::NAVIGATION_STATE_SMART;

	// try to publish the static LED for the first 10s
	// this avoid the problem if a LED driver did not subscribe to the topic yet
	if (hrt_absolute_time() < 10 * 1000000 || restore_position_lights) {
		// set the base color for right LED
		_led_control.led_mask = BOARD_RIGHT_LED_MASK;
		_led_control.color = led_control_s::COLOR_GREEN;
		_led_control.mode = led_control_s::MODE_ON;
		publish();

		// set the base color for front LED
		_led_control.led_mask = BOARD_FRONT_LED_MASK;
		_led_control.color = led_control_s::COLOR_WHITE;
		_led_control.mode = led_control_s::MODE_ON;
		publish();

		// set the base color for left LED
		_led_control.led_mask = BOARD_LEFT_LED_MASK;
		_led_control.color = led_control_s::COLOR_RED;
		_led_control.mode = led_control_s::MODE_ON;
		publish();
	}

	matrix::Eulerf euler = matrix::Quatf(_vehicle_attitude.q);
	float current_heading = euler.psi();
	float delta_heading = current_heading - _smart_heading.smart_heading_ref;
	int sector = roundf(delta_heading / (M_PI_F / 3.0f));

	// set the led mask for the status led which are the back LED
	_led_control.led_mask = BOARD_REAR_LED_MASK;

	// TODO: need reworking with new LED definition this does only work on H520
	// choose color depending on the nav state
	if (nav_state == vehicle_status_s::NAVIGATION_STATE_SMART
	    || (_old_nav_state == vehicle_status_s::NAVIGATION_STATE_SMART && _old_sector != sector)) {
		// set green position light
		_led_control.led_mask = (1 << ((1 + sector) % 6 >= 0 ? (1 + sector) % 6 : 1 + sector + 6));
		_led_control.color = led_control_s::COLOR_GREEN;
		_led_control.mode = led_control_s::MODE_ON;
		publish();
		// set white forward LEDs
		_led_control.led_mask = (1 << ((2 + sector) % 6 >= 0 ? (2 + sector) % 6 : 2 + sector + 6)) | (1 << ((
						3 + sector) % 6 >= 0 ? (3 + sector) % 6 : 3 + sector + 6));
		_led_control.color = led_control_s::COLOR_WHITE;
		_led_control.mode = led_control_s::MODE_ON;
		publish();
		// set red position light
		_led_control.led_mask = (1 << ((4 + sector) % 6 >= 0 ? (4 + sector) % 6 : 4 + sector + 6));
		_led_control.color = led_control_s::COLOR_RED;
		_led_control.mode = led_control_s::MODE_ON;
		publish();
		// set purple back LEDs
		_led_control.led_mask = (1 << ((0 + sector) % 6 >= 0 ? (0 + sector) % 6 : 0 + sector + 6)) | (1 << ((
						5 + sector) % 6 >= 0 ? (5 + sector) % 6 : 5 + sector + 6));
		_led_control.color = led_control_s::COLOR_PURPLE;
		_led_control.mode = led_control_s::MODE_ON;


	} else if (nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_RTL
		   || nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LAND) {
		_led_control.color = led_control_s::COLOR_YELLOW;

	} else if (nav_state == vehicle_status_s::NAVIGATION_STATE_ALTCTL) {
		_led_control.color = led_control_s::COLOR_BLUE;

	} else if (nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION) {
		_led_control.color = led_control_s::COLOR_GREEN;

	} else {
		_led_control.color = led_control_s::COLOR_PURPLE;
	}

	// blink if no GPS and home are set
	if (gps_lock_valid && home_position_valid) {
		_led_control.mode = led_control_s::MODE_ON;

	} else {
		_led_control.mode = led_control_s::MODE_BLINK_NORMAL;
	}

	// handle battery warnings, once a state is reached it can not be reset
	if (_battery_status.warning == battery_status_s::BATTERY_WARNING_CRITICAL || _critical_battery) {
		_led_control.color = led_control_s::COLOR_RED;
		_led_control.mode = led_control_s::MODE_BLINK_FAST;
		_critical_battery = true;

	} else if (_battery_status.warning == battery_status_s::BATTERY_WARNING_LOW || _low_battery) {
		_led_control.color = led_control_s::COLOR_RED;
		_led_control.mode = led_control_s::MODE_FLASH;
		_low_battery = true;
	}

	if (nav_state != _old_nav_state || gps_lock_valid != _old_gps_lock_valid
	    || home_position_valid != _old_home_position_valid || _battery_status.warning != _old_battery_status_warning
	    || (_old_nav_state == vehicle_status_s::NAVIGATION_STATE_SMART && _old_sector != sector)) {
		publish();
	}

	// copy actual state
	_old_sector = sector;
	_old_nav_state = nav_state;
	_old_gps_lock_valid = gps_lock_valid;
	_old_home_position_valid = home_position_valid;
	_old_battery_status_warning = _battery_status.warning;
}

} /* namespace status */
} /* namespace events */
