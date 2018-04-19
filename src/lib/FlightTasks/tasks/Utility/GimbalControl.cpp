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

GimbalControl::GimbalControl()
{
	_mnt_yaw_ctl = param_find("MIS_MNT_YAW_CTL");
}

void GimbalControl::_PointOfInterest(const matrix::Vector3f poi, const matrix::Vector3f position, const float yaw)
{
	Vector2f position_to_poi_vec(poi(0) - position(0), poi(1) - position(1));
	float position_to_poi_vec_z = poi(2) - position(2);
	position_to_poi_vec.normalized();

	mount_orientation_s mount_orientation = {};
	mount_orientation.timestamp = hrt_absolute_time();
	/* roll */
	mount_orientation.attitude_euler_angle[0] = 0.0f;

	/* pitch: calculate angle between ground and vehicle */
	mount_orientation.attitude_euler_angle[1] = -atan2f(position_to_poi_vec_z, position_to_poi_vec.length());

	/* yaw: calculate the angle between vector v = (poi - position) and u = (1, 0)
	 * alpha = arccos((u * v) / (||u|| * ||v||)) = arccos(v(0) / ||v||)
	 * the sign of the angle is given by sign(u x v) = sign(u(0) * v(1) + u(1) * v(0)) = sign(v(1))
	 * substract vehicle yaw because the gimbal neutral position is in the UAV heading direction */
	mount_orientation.attitude_euler_angle[2] = math::sign(position_to_poi_vec(1)) * wrap_pi(acosf(position_to_poi_vec(
				0) / position_to_poi_vec.norm())) - yaw;
	_PublishMountOrientation(mount_orientation);
}

void GimbalControl::_PublishMountOrientation(struct mount_orientation_s &mount_orientation)
{
	int instance = 1;
	orb_publish_auto(ORB_ID(mount_orientation), &_pub_mount_orientation, &mount_orientation, &instance, ORB_PRIO_DEFAULT);
}