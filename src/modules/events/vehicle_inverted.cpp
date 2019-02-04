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
 * @file vehicle_inverted.cpp
 * Detect vehicle inverted state to set the landing gear accordig to the RC switch
 *
 * @author Simone Guscetti <simone@yuneecresearch.com>
 *
 */

#include "vehicle_inverted.h"

#include <px4_log.h>

#include <DevMgr.hpp>
#include <drivers/drv_gpio.h>
#include <drivers/drv_pwm_output.h>

namespace events
{
namespace inverted
{

VehicleInverted::VehicleInverted(const events::SubscriberHandler &subscriber_handler)
	: _subscriber_handler(subscriber_handler) {}

bool VehicleInverted::check_for_updates()
{
	bool got_updates = false;

	if (_subscriber_handler.manual_control_sp_updated()) {
		orb_copy(ORB_ID(manual_control_setpoint), _subscriber_handler.get_manual_control_setpoint_sub(), &_manual_control_sp);
		got_updates = true;
	}

	if (_subscriber_handler.vehicle_land_detcted_updated()) {
		orb_copy(ORB_ID(vehicle_land_detected), _subscriber_handler.get_vehicle_land_detected_sub(), &_land_detector);
		got_updates = true;
	}

	if (_subscriber_handler.vehicle_status_flags_updated()) {
		orb_copy(ORB_ID(vehicle_status_flags), _subscriber_handler.get_vehicle_status_flags_sub(), &_status_flags);
		got_updates = true;
	}

	return got_updates;
}

void VehicleInverted::process()
{
	if (!check_for_updates()) {
		return;
	}

	if (_land_detector.inverted && !_status_flags.condition_calibration_enabled) {

		if (!_pwm_armed) {
			if (arm_pwm(true) == PX4_OK) {
				_pwm_armed = true;
			}
		}

		if (_gear_pos_prev != _manual_control_sp.gear_switch) {
			_landing_gear.landing_gear =
				(_manual_control_sp.gear_switch == manual_control_setpoint_s::SWITCH_POS_ON) ?
				landing_gear_s::GEAR_UP :
				landing_gear_s::GEAR_DOWN;

			publish();
		}

	} else if (_pwm_armed) {

		if (arm_pwm(false) == PX4_OK) {
			_pwm_armed = false;
		}
	}

	_gear_pos_prev = _manual_control_sp.gear_switch;
}

void VehicleInverted::publish()
{
	_landing_gear.timestamp = hrt_absolute_time();

	if (_landing_gear_pub != nullptr) {
		orb_publish(ORB_ID(landing_gear), _landing_gear_pub, &_landing_gear);

	} else {
		_landing_gear_pub =  orb_advertise(ORB_ID(landing_gear), &_landing_gear);
	}
}

int VehicleInverted::arm_pwm(bool arm)
{
	DriverFramework::DevHandle fmu_h;
	DriverFramework::DevMgr::getHandle(PX4FMU_DEVICE_PATH, fmu_h);

	if (!fmu_h.isValid()) {
		PX4_WARN("FMU: getHandle fail\n");
		return PX4_ERROR;
	}

	if (arm) {
		if (fmu_h.ioctl(PWM_SERVO_ARM, 0) != OK) {
			PX4_ERR("PWM_SERVO_ARM fail");
			return PX4_ERROR;
		}

	} else {
		if (fmu_h.ioctl(PWM_SERVO_DISARM, 0) != OK) {
			PX4_ERR("PWM_SERVO_DISARM fail");
			return PX4_ERROR;
		}

	}

	return PX4_OK;
}

} /* namespace inverted */
} /* namespace events */
