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
 * @file GimbalControl.hpp
 *
 * This Class is used to point the gimbal towards the point of interest.
 */

#pragma once

#include <matrix/matrix/math.hpp>
#include <px4_module_params.h>
#include <uORB/topics/vehicle_command.h>

class GimbalControl : public ModuleParams
{
public:
	GimbalControl(ModuleParams *parent);
	~GimbalControl() = default;

	/**
	 * Calculates gimbal orientation to point towards a point of interest
	 * @param poi: point of iterest in local NED coordinates
	 * @param position: UAV position in local NED coordinates
	 * @param yaw: UAV yaw in radiants
	 */
	void pointOfInterest(const matrix::Vector3f &poi, const matrix::Vector3f &position, const float yaw);


private:

	orb_advert_t _pub_vehicle_command = nullptr;

	int32_t _sys_id = 0;
	int32_t _cmp_id = 0;

	/** publishes on topic vehicle_command_flight_task */
	void _publishVehicleCommand(const vehicle_command_s &vehicle_command);

	DEFINE_PARAMETERS((ParamInt<px4::params::MIS_MNT_YAW_CTL>) _param_mnt_yaw_ctl)
};
