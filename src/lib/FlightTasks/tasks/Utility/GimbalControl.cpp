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

/**
 * @file GimbalControl.cpp
 */

#include "GimbalControl.hpp"
#include <mathlib/mathlib.h>
#include <drivers/drv_hrt.h>

using namespace matrix;

GimbalControl::GimbalControl():
	ModuleParams(nullptr)
{
	param_t param_sys_id = param_find("MAV_SYS_ID");
	param_t param_comp_id = param_find("MAV_COMP_ID");

	param_get(param_sys_id, &_sys_id);
	param_get(param_comp_id, &_cmp_id);
}

void GimbalControl::pointOfInterest(const Vector3f &poi, const Vector3f &position, const float yaw)
{
	ModuleParams::updateParams();

	vehicle_command_s vehicle_command = {};
	vehicle_command.timestamp = hrt_absolute_time();
	vehicle_command.target_system = _sys_id;
	vehicle_command.target_component = _cmp_id;
	vehicle_command.command = vehicle_command_s::VEHICLE_CMD_DO_MOUNT_CONTROL;


	if (_param_mnt_yaw_ctl.get() == 0) {

		/* yaw control disabled, the vehicle will yaw towards the point of interest
		 * with the gimbal in neutral position */
		vehicle_command.param7 = vehicle_command_s::VEHICLE_MOUNT_MODE_NEUTRAL;

	} else {

		/* yaw control enabled, the gimbal will rotate towards the point of interest */
		vehicle_command.param7 = vehicle_command_s::VEHICLE_MOUNT_MODE_MAVLINK_TARGETING;

		Vector2f position_to_poi_vec(poi(0) - position(0), poi(1) - position(1));
		float position_to_poi_vec_z = poi(2) - position(2);
		position_to_poi_vec.normalized();

		/* roll */
		vehicle_command.param2 = 0.0f;

		/* pitch: calculate angle between ground and vehicle */
		vehicle_command.param1 = math::degrees(-atan2f(position_to_poi_vec_z, position_to_poi_vec.length()));

		/* yaw: calculate the angle between vector v = (poi - position) and u = (1, 0)
		 * alpha = arccos((u * v) / (||u|| * ||v||)) = arccos(v(0) / ||v||)
		 * the sign of the angle is given by sign(u x v) = sign(u(0) * v(1) + u(1) * v(0)) = sign(v(1))
		 * substract vehicle yaw because the gimbal neutral position is in the UAV heading direction */
		vehicle_command.param3 = math::degrees(math::sign(position_to_poi_vec(1)) * wrap_pi(acosf(position_to_poi_vec(
				0) / position_to_poi_vec.norm())) - yaw);
	}

	_publishVehicleCommand(vehicle_command);
}

void GimbalControl::_publishVehicleCommand(struct vehicle_command_s &vehicle_command)
{

	if (_pub_vehicle_command != nullptr) {
		orb_publish(ORB_ID(vehicle_command), _pub_vehicle_command, &vehicle_command);

	} else {
		_pub_vehicle_command = orb_advertise(ORB_ID(vehicle_command), &vehicle_command);
	}
}
