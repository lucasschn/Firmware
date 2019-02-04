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
 * @file ObstacleAvoidance.cpp
 */

#include "ObstacleAvoidance.hpp"
#include <mathlib/mathlib.h>
#include <drivers/drv_hrt.h>
#include <float.h>

using namespace matrix;

// Time in us that the no obstacle ahead condition has to be true to exit obstacle avoidance lock in
static constexpr uint64_t OBSTACLE_LOCK_EXIT_TRIGGER_TIME_US = 1000000;
// Timeout in us for obstacle avoidance sonar data to get considered invalid
static constexpr uint64_t DISTANCE_STREAM_TIMEOUT_US = 500000;
// Minimum altitude to start obstacle avoidance
static constexpr float MINIMUM_ALTITUDE = 2.0f;
// Minimum yaw change to exit obstacle avoidance lock
static constexpr float UNLOCK_YAW = 30.0f;

#define SIGMA_NORM	0.001f

ObstacleAvoidance::ObstacleAvoidance(ModuleParams *parent) :
	ModuleParams(parent),
	_obstacle_lock_hysteresis(false)
{
	_obstacle_distance_sub = orb_subscribe(ORB_ID(obstacle_distance));

	for (unsigned int i = 0; i < ORB_MULTI_MAX_INSTANCES; i++) {
		_distance_sensor_sub[i] = orb_subscribe_multi(ORB_ID(distance_sensor), i);
	}

	_home_position_sub = orb_subscribe(ORB_ID(home_position));
	_manual_control_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	_vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));

	_obstacle_lock_hysteresis.set_hysteresis_time_from(true, OBSTACLE_LOCK_EXIT_TRIGGER_TIME_US);

}

ObstacleAvoidance::~ObstacleAvoidance()
{
	orb_unsubscribe(_obstacle_distance_sub);

	for (unsigned int i = 0; i < ORB_MULTI_MAX_INSTANCES; i++) {
		orb_unsubscribe(_distance_sensor_sub[i]);
	}

	orb_unsubscribe(_home_position_sub);
	orb_unsubscribe(_manual_control_sub);
	orb_unsubscribe(_vehicle_status_sub);
}

void ObstacleAvoidance::_updateSubscriptions()
{

	bool updated = false;

	if (_sonar_sub_index >= 0) {
		orb_check(_distance_sensor_sub[_sonar_sub_index], &updated);

		if (updated) {
			orb_copy(ORB_ID(distance_sensor), _distance_sensor_sub[_sonar_sub_index], &_sonar_measurement);
		}

	} else {
		_sonar_sub_index = _getSonarSubIndex(_distance_sensor_sub);
	}

	orb_check(_obstacle_distance_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(obstacle_distance), _obstacle_distance_sub, &_obstacle_distance);
	}

	orb_check(_home_position_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(home_position), _home_position_sub, &_home_position);
	}

	orb_check(_manual_control_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(manual_control_setpoint), _manual_control_sub, &_manual);
	}

	orb_check(_vehicle_status_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &_vehicle_status);
	}
}


int ObstacleAvoidance::_getSonarSubIndex(const int *subs)
{
	for (unsigned int instance = 0; instance < ORB_MULTI_MAX_INSTANCES; instance++) {
		bool updated = false;
		orb_check(subs[instance], &updated);

		if (updated) {
			distance_sensor_s report;
			orb_copy(ORB_ID(distance_sensor), subs[instance], &report);

			// only use the instace which has the correct forward orientation
			if (report.orientation == distance_sensor_s::ROTATION_FORWARD_FACING) {
				return instance;
			}
		}
	}

	return PX4_ERROR;
}


void ObstacleAvoidance::stopInFront(const Vector3f &position, const float yaw, const Vector3f &desired_vel_sp,
				    const float desired_yawspeed_sp)
{
	_updateSubscriptions();

	// start sense&stop if the ST16 switch is in ON or MIDDLE position and if the vehicle
	// is in POSCTL mode
	if (((_manual.obsavoid_switch == manual_control_setpoint_s::SWITCH_POS_ON)
	     || (_manual.obsavoid_switch == manual_control_setpoint_s::SWITCH_POS_MIDDLE))
	    && (_vehicle_status.nav_state == _vehicle_status.NAVIGATION_STATE_POSCTL)) {

		struct vehicle_trajectory_waypoint_s traj_waypoint = {};
		traj_waypoint.timestamp = hrt_absolute_time();
		traj_waypoint.type = 0;
		Vector3f(NAN, NAN, NAN).copyTo(traj_waypoint.waypoints[0].position);
		desired_vel_sp.copyTo(traj_waypoint.waypoints[0].velocity);
		Vector3f(NAN, NAN, NAN).copyTo(traj_waypoint.waypoints[0].acceleration);
		traj_waypoint.waypoints[0].yaw = NAN;
		traj_waypoint.waypoints[0].yaw_speed = desired_yawspeed_sp;
		traj_waypoint.waypoints[0].point_valid = false;

		// default set obstacle_distance distance unknown
		_min_obstacle_distance = (float)UINT16_MAX;

		const float altitude_above_home = -position(2) + _home_position.z;

		const bool obstacle_distance_valid = hrt_elapsed_time((hrt_abstime *)&_obstacle_distance.timestamp) <
						     DISTANCE_STREAM_TIMEOUT_US;

		// if obstacle_distance is published, we always want to calculate the minimum distance to obstacle
		if (obstacle_distance_valid) {
			_setMinimumDistance(yaw);
		}

		// keep a minimum braking distance of start_braking_distance, otherwise give the vehicle at least 1s time to brake
		float safety_margin = 1.0f;
		const float brake_distance = math::max(_start_braking_distance.get(), _vel_max_xy_param.get() + safety_margin);

		// tap specific: fuse obstacle_distance from RealSense and sonar
		_min_obstacle_distance = _fuseObstacleDistanceSonar(altitude_above_home, safety_margin, brake_distance);

		// anything under the brake distance is considered an obstacle. The vehicle altitude above ground should be at least 1.5m not to detect ground as an obstacle
		const bool obstacle_ahead = _min_obstacle_distance < brake_distance && altitude_above_home > MINIMUM_ALTITUDE;

		if (obstacle_ahead) {
			// vehicle just detected an obstacle
			_obstacle_lock_hysteresis.set_state_and_update(true);
			_yaw_obstacle_lock = yaw;
		}

		// get velocity setpoint in heading frame
		matrix::Quatf q_yaw = matrix::AxisAnglef(matrix::Vector3f(0.0f, 0.0f, 1.0f), yaw);
		matrix::Vector3f vel_sp_heading = q_yaw.conjugate_inversed(matrix::Vector3f(desired_vel_sp(0), desired_vel_sp(1),
						  0.0f));

		// adjust velocity setpoint based on obstacle_distance
		if (_obstacle_lock_hysteresis.get_state()) {
			traj_waypoint.waypoints[0].point_valid = true;

			// stay in obstacle lock when trying to go forwards without yawing 30 degree away from the last obstacle
			_obstacle_lock_hysteresis.set_state_and_update(vel_sp_heading(0) > FLT_EPSILON &&
					fabsf(yaw - _yaw_obstacle_lock) > math::radians(UNLOCK_YAW));

			// we don't allow movement perpendicular to heading direction
			vel_sp_heading(1) = 0.0f;

			// default: we only allow movement in heading direction
			matrix::Vector3f vel_sp_tmp = q_yaw.conjugate(vel_sp_heading);
			traj_waypoint.waypoints[0].velocity[0] = vel_sp_tmp(0);
			traj_waypoint.waypoints[0].velocity[1] = vel_sp_tmp(1);

			if (vel_sp_heading(0) > 0.0f) {
				// vehicle wants to fly towards obstacle

				if (_min_obstacle_distance <= safety_margin) {
					// vehicle is already safety_margin close to obstacle. Don't move forward

					traj_waypoint.waypoints[0].velocity[0] = 0.0f;
					traj_waypoint.waypoints[0].velocity[1] = 0.0f;

				} else {
					// vehicle wants to move forward but is more than safety margin away

					// limit the speed linearly from max velocity to zero over braking distance + safety margin
					const float m = _vel_max_xy_param.get() / (brake_distance - safety_margin); // slope
					const float speed_limit =  m * (_min_obstacle_distance - safety_margin);

					if (vel_sp_heading(0) > speed_limit && vel_sp_heading(0) >= SIGMA_NORM) {

						// desired heading velocity is above speed limit
						traj_waypoint.waypoints[0].velocity[0] = vel_sp_tmp(0) / vel_sp_tmp.length() * speed_limit;
						traj_waypoint.waypoints[0].velocity[1] = vel_sp_tmp(1) / vel_sp_tmp.length() * speed_limit;
					}
				}
			}
		}

		// publish waypoint
		if (_trajectory_waypoint_pub != nullptr) {
			orb_publish(ORB_ID(vehicle_trajectory_waypoint), _trajectory_waypoint_pub, &traj_waypoint);

		} else {
			_trajectory_waypoint_pub = orb_advertise(ORB_ID(vehicle_trajectory_waypoint), &traj_waypoint);
		}

	} else {
		// release lock if obstacle avoidance off
		_obstacle_lock_hysteresis.set_state_and_update(false);
	}
}

void ObstacleAvoidance::_setMinimumDistance(const float yaw)
{

	// find front direction in the array of obsacle positions
	float yaw_deg = math::degrees(yaw);

	if ((yaw_deg <= FLT_EPSILON) || (yaw_deg >= 180.0f)) {
		yaw_deg += 360.0f;
	}

	// if increment not set, put it to the minimum value to represent obstacles 360 degrees around the MAV
	_obstacle_distance.increment = (_obstacle_distance.increment == 0) ? obstacle_distance_s::MINIMUM_INCREMENT :
				       _obstacle_distance.increment;

	// array resolution defined by the increment, index 0 is always global north
	const int index = (int)floorf(yaw_deg / (float)_obstacle_distance.increment);

	const int distances_array_length = sizeof(_obstacle_distance.distances) / sizeof(uint16_t);

	// consider only 30 degrees in front of the MAV
	for (int idx = -3; idx < 3; ++idx) {
		// exclude measurements greater than max_distance+1 because it means there is no obstacle ahead
		int shifted_index = index + idx;

		if (shifted_index < 0) {
			shifted_index = distances_array_length + shifted_index;

		} else if (shifted_index >= distances_array_length) {
			shifted_index = shifted_index % distances_array_length;
		}

		if (_obstacle_distance.distances[shifted_index] < (_obstacle_distance.max_distance + 1)) {
			// covert from centimeters to meters
			_min_obstacle_distance = math::min(_min_obstacle_distance,
							   (float)_obstacle_distance.distances[shifted_index] * 0.01f);
		}
	}
}

float ObstacleAvoidance::_fuseObstacleDistanceSonar(float altitude_above_home, float &safety_margin,
		const float brake_distance)
{
	// by default we use only obstacle_distance data
	float minimum_distance_fused = _min_obstacle_distance;

	// obstacle_distance (RealSense): anything under the brake distance is considered an obstalce
	const bool obstacle_ahead = _min_obstacle_distance < brake_distance && altitude_above_home > MINIMUM_ALTITUDE;

	// sonar is pointing forward, data stream is running, omit floor detection in low altitude
	const bool valid_sonar_measurement = _sonar_measurement.orientation == distance_sensor_s::ROTATION_FORWARD_FACING &&
					     hrt_elapsed_time((hrt_abstime *)&_sonar_measurement.timestamp) < DISTANCE_STREAM_TIMEOUT_US &&
					     altitude_above_home > MINIMUM_ALTITUDE;
	// sonar: anything but maximum distance measurement is considered an obstacle
	const bool obstacle_ahead_sonar = _sonar_measurement.current_distance < _sonar_measurement.max_distance;

	if (!obstacle_ahead && valid_sonar_measurement && obstacle_ahead_sonar) {

		// sonar only detected obstacle
		minimum_distance_fused = _sonar_measurement.current_distance;

		// if only the sonar is active, increase safety margin to max_distance so that the UAV
		// starts braking as soon as the obstacle is detected
		safety_margin = _sonar_measurement.max_distance;

	} else if (obstacle_ahead && obstacle_ahead_sonar && (_sonar_measurement.current_distance <= safety_margin)) {
		// if both sonar and obstacle_distance detect obstacle, then only consider
		// sonar if sonar distant measurement is below safety margin
		minimum_distance_fused = _sonar_measurement.current_distance;

	}

	return minimum_distance_fused;
}
