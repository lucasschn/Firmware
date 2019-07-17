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
 * @file ObstacleAvoidance.hpp
 *
 * lib to stop in front of obstacles
 */

#pragma once

#include <px4_module_params.h>
#include <matrix/matrix/math.hpp>
#include <lib/hysteresis/hysteresis.h>

#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/obstacle_distance.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_trajectory_waypoint.h>


class ObstacleAvoidance : public ModuleParams
{
public:
	ObstacleAvoidance(ModuleParams *parent);
	~ObstacleAvoidance();

	/**
	 * Method to publish velocity setpoints to stop in front of an obstacle
	 * @param vehicle local position
	 * @param vehicle heading
	 * @param desired velocity setpoint
	 * @param desired yaw speed setpoint
	 *
	 */
	void stopInFront(const matrix::Vector3f &_position, const float _yaw, const matrix::Vector3f &_desired_vel_sp,
			 const float _desired_yawspeed_sp);

private:

	int _distance_sensor_sub[ORB_MULTI_MAX_INSTANCES];		/**< distance_sensor subscritption */
	int _sonar_sub_index = -1; /** index of the sonar instance facing forward */
	int _obstacle_distance_sub = -1; /**< obstacle_distance subscritption*/
	int _home_position_sub = -1; /**< home_position subscritption*/
	int _manual_control_sub = -1; /**< manual_control_setpoint subscription */
	int _vehicle_status_sub = -1; /**< vehicle_status subscription */

	float _min_obstacle_distance = 0.0f; /**< minimum distance to an obstacle */
	float _yaw_obstacle_lock = 0.0f; /**< */

	struct distance_sensor_s _sonar_measurement = {}; /**< sonat distance sensor data */
	struct home_position_s _home_position = {}; /**< home position */
	struct manual_control_setpoint_s _manual = {}; /**< manual control setpoint */
	struct obstacle_distance_s _obstacle_distance = {}; /**< obstacle distance data*/
	struct vehicle_status_s _vehicle_status = {}; /**< vehicle status */

	systemlib::Hysteresis _obstacle_lock_hysteresis; /**< hysteresis to lock the vehicle in front of the obstacle */

	orb_advert_t _trajectory_waypoint_pub = nullptr; /**< vehicle_trajectory_waypoint publisher */

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::MPC_DIST_BRAKE>)
		_start_braking_distance, /**< distance from an obstacle at which braking starts */
		(ParamFloat<px4::params::MPC_XY_VEL_MAX>) _vel_max_xy_param /**< maximum horizontal velocity */
	);

	void _setMinimumDistance(const float yaw); /**< get obstacle minimum distance from cameras */
	float _fuseObstacleDistanceSonar(float altitude_above_home, float &safety_margin,
					 const float brake_distance); /**< function to fuse distance data from RealSense and Sonar >*/
	int _getSonarSubIndex(const int *subs); /**< get the sonar instance number in uORB subscription >*/
	void _updateSubscriptions(); /**< update uORB subscriptions */
};