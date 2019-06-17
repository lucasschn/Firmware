/****************************************************************************
 *
 *   Copyright (c) 2013-2018 PX4 Development Team. All rights reserved.
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
 * @file rtl.cpp
 *
 * Helper class to access RTL
 *
 * @author Julian Oes <julian@oes.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#include "rtl.h"
#include "navigator.h"

#include <math.h>

#include <drivers/drv_hrt.h>
#include <uORB/topics/vehicle_command.h>

static constexpr float DELAY_SIGMA = 0.01f;

RTL::RTL(Navigator *navigator) :
	MissionBlock(navigator),
	ModuleParams(navigator)
{
}

RTL::~RTL()
{
	orb_unadvertise(_rtl_time_estimate_pub);
}

void
RTL::on_inactive()
{
	using namespace time_literals;

	// Reset RTL state.
	_rtl_state = RTL_STATE_NONE;

	// Limit calculation and publishing frequency
	if ((hrt_absolute_time() - _rtl_time_estimate.timestamp) > 1_s) {
		update_return_location();
		calc_and_pub_rtl_time_estimate();
	}
}

int
RTL::rtl_type() const
{
	return _param_rtl_type.get();
}

void
RTL::on_activation()
{
	set_current_position_item(&_mission_item);
	struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
	mission_apply_limitation(_mission_item);
	mission_item_to_position_setpoint(_mission_item, &pos_sp_triplet->current);
	pos_sp_triplet->previous.valid = false;
	pos_sp_triplet->next.valid = false;

	// set home position to either GS or takeoff location
	update_return_location();

	// go directly to home if obstacle avoidance is used
	const bool obstacle_avoidance_switch_on = _navigator->get_manual_setpoint()->obsavoid_switch ==
			manual_control_setpoint_s::SWITCH_POS_ON;
	const bool obstacle_avoidance_running = _navigator->get_trajectory_waypoint()->point_valid == true;

	/* for safety reasons don't go into RTL if landed */
	if (_navigator->get_land_detected()->landed) {
		// For safety reasons don't go into RTL if landed.
		_rtl_state = RTL_STATE_LANDED;
		mavlink_log_info(_navigator->get_mavlink_log_pub(), "Already landed, not executing RTL");
		// otherwise start RTL by braking first

	} else if (obstacle_avoidance_running && obstacle_avoidance_switch_on) {
		_rtl_state = RTL_STATE_HOME;
		mavlink_log_info(_navigator->get_mavlink_log_pub(), "Flying straight to home");

	} else if ((rtl_type() == RTL_LAND) && _navigator->on_mission_landing()) {
		// RTL straight to RETURN state, but mission will takeover for landing.

	} else {
		_rtl_state = RTL_STATE_BRAKE;
	}

	set_rtl_item();

	{
		// Disable camera distance based trigger
		vehicle_command_s cmd = {};
		cmd.command = vehicle_command_s::VEHICLE_CMD_DO_TRIGGER_CONTROL;
		// Pause trigger
		cmd.param1 = -1.0f;
		cmd.param3 = 1.0f;
		_navigator->publish_vehicle_cmd(&cmd);
	}

	{
		// Disable camera time based image capture
		vehicle_command_s cmd = {};
		cmd.command = vehicle_command_s::VEHICLE_CMD_IMAGE_STOP_CAPTURE;
		// Pause trigger
		cmd.param1 = 0.0f;
		_navigator->publish_vehicle_cmd(&cmd);
	}
}

void
RTL::on_active()
{
	using namespace time_literals;

	if (_rtl_state != RTL_STATE_LANDED && is_mission_item_reached()) {
		advance_rtl();
		set_rtl_item();
	}

	// Limit calculation and publishing frequency
	if ((hrt_absolute_time() - _rtl_time_estimate.timestamp) > 1_s) {
		calc_and_pub_rtl_time_estimate();
	}
}

void
RTL::set_return_alt_min(bool min)
{
	_rtl_alt_min = min;
}

void
RTL::set_rtl_item()
{
	// RTL_TYPE: mission landing.
	// Landing using planned mission landing, fly to DO_LAND_START instead of returning HOME.
	// Do nothing, let navigator takeover with mission landing.
	if (rtl_type() == RTL_LAND) {
		if (_rtl_state > RTL_STATE_CLIMB) {
			if (_navigator->start_mission_landing()) {
				mavlink_and_console_log_info(_navigator->get_mavlink_log_pub(), "RTL: using mission landing");
				return;

			} else {
				// Otherwise use regular RTL.
				mavlink_log_critical(_navigator->get_mavlink_log_pub(), "RTL: unable to use mission landing");
			}
		}
	}

	_navigator->set_can_loiter_at_sp(false);

	const home_position_s home = _return_location;
	const vehicle_global_position_s &gpos = *_navigator->get_global_position();

	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	// Check if we are pretty close to home already.
	const float home_dist = get_distance_to_next_waypoint(home.lat, home.lon, gpos.lat, gpos.lon);

	// Compute the return altitude
	float return_alt = get_rtl_altitude();

	// We are close to home, limit climb to min
	// if (home_dist < _param_rtl_min_dist.get()) {
	// 	return_alt = home.alt + _param_descend_alt.get();
	// }

	bool home_close = (home_dist < _param_rtl_min_dist.get());
	bool home_altitude_close = (fabsf(gpos.alt - home.alt) < _param_rtl_min_dist.get());

	// Compute the loiter altitude.
	const float loiter_altitude = math::min(home.alt + _param_descend_alt.get(), gpos.alt);

	switch (_rtl_state) {
	case RTL_STATE_BRAKE: {  // NOTE: Yuneec-specific state

			/* slow down before climbing */
			set_brake_item(&_mission_item);

			break;

		}

	case RTL_STATE_CLIMB: {

			_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
			_mission_item.lat = gpos.lat;
			_mission_item.lon = gpos.lon;
			_mission_item.altitude_is_relative = false;
			_mission_item.altitude = return_alt;
			_mission_item.yaw = _navigator->get_local_position()->yaw;
			_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
			_mission_item.time_inside = 0.0f;
			_mission_item.autocontinue = true;
			_mission_item.origin = ORIGIN_ONBOARD;
			_mission_item.landing_gear = (home_close && home_altitude_close) ? landing_gear_s::GEAR_DOWN : landing_gear_s::GEAR_UP;
			_mission_item.force_velocity = false;

			mavlink_and_console_log_info(_navigator->get_mavlink_log_pub(), "RTL: climb to %d m (%d m above home)",
						     (int)ceilf(return_alt), (int)ceilf(return_alt - home.alt));
			break;
		}

	case RTL_STATE_PRE_RETURN: {

			_mission_item.lat = gpos.lat;
			_mission_item.lon = gpos.lon;

			if (home_close) {
				_mission_item.yaw = home.yaw;

			} else {
				// use current heading to home
				_mission_item.yaw = get_bearing_to_next_waypoint(
							    gpos.lat, gpos.lon,
							    home.lat, home.lon);

			}

			_mission_item.loiter_radius = _navigator->get_loiter_radius();
			_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
			_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
			_mission_item.time_inside = 0.0f;
			_mission_item.autocontinue = true;
			_mission_item.origin = ORIGIN_ONBOARD;
			_mission_item.landing_gear = (home_close && home_altitude_close) ? landing_gear_s::GEAR_DOWN : landing_gear_s::GEAR_UP;
			break;
		}

	case RTL_STATE_RETURN: {

			// Don't change altitude.
			_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
			_mission_item.lat = home.lat;
			_mission_item.lon = home.lon;
			_mission_item.altitude = return_alt;
			_mission_item.altitude_is_relative = false;

			// Use home yaw if close to home
			if (home_close) {
				_mission_item.yaw = home.yaw;

			} else {
				// Use current heading to home.
				_mission_item.yaw = get_bearing_to_next_waypoint(gpos.lat, gpos.lon, home.lat, home.lon);
			}

			_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
			_mission_item.time_inside = 0.0f;
			_mission_item.autocontinue = true;
			_mission_item.origin = ORIGIN_ONBOARD;
			_mission_item.landing_gear = (home_close && home_altitude_close) ? landing_gear_s::GEAR_DOWN : landing_gear_s::GEAR_UP;

			mavlink_and_console_log_info(_navigator->get_mavlink_log_pub(), "RTL: return at %d m (%d m above home)",
						     (int)ceilf(_mission_item.altitude), (int)ceilf(_mission_item.altitude - home.alt));

			break;
		}

	case RTL_STATE_AFTER_RETURN: {
			_mission_item.lat = home.lat;
			_mission_item.lon = home.lon;
			// don't change altitude
			_mission_item.yaw = home.yaw;

			_mission_item.loiter_radius = _navigator->get_loiter_radius();
			_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
			_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
			_mission_item.time_inside = 0.0f;
			_mission_item.autocontinue = true;
			_mission_item.origin = ORIGIN_ONBOARD;
			_mission_item.landing_gear = (home_close && home_altitude_close) ? landing_gear_s::GEAR_DOWN : landing_gear_s::GEAR_UP;
			break;
		}

	case RTL_STATE_TRANSITION_TO_MC: {
			set_vtol_transition_item(&_mission_item, vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC);
			break;
		}

	case RTL_STATE_DESCEND: {
			_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
			_mission_item.lat = home.lat;
			_mission_item.lon = home.lon;
			_mission_item.altitude = loiter_altitude;
			_mission_item.altitude_is_relative = false;

			// Except for vtol which might be still off here and should point towards this location.
			const float d_current = get_distance_to_next_waypoint(gpos.lat, gpos.lon, _mission_item.lat, _mission_item.lon);

			if (_navigator->get_vstatus()->is_vtol && (d_current > _navigator->get_acceptance_radius())) {
				_mission_item.yaw = get_bearing_to_next_waypoint(gpos.lat, gpos.lon, _mission_item.lat, _mission_item.lon);

			} else {
				_mission_item.yaw = home.yaw;
			}

			_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
			_mission_item.time_inside = 0.0f;
			_mission_item.autocontinue = true;
			_mission_item.origin = ORIGIN_ONBOARD;
			_mission_item.landing_gear = (home_close && home_altitude_close) ? landing_gear_s::GEAR_DOWN : landing_gear_s::GEAR_UP;

			// Disable previous setpoint to prevent drift.
			pos_sp_triplet->previous.valid = false;
			break;
		}

	case RTL_STATE_LOITER: {
			const bool autoland = (_param_land_delay.get() > FLT_EPSILON);

			// Don't change altitude.
			_mission_item.lat = home.lat;
			_mission_item.lon = home.lon;
			_mission_item.altitude = loiter_altitude;
			_mission_item.altitude_is_relative = false;
			_mission_item.yaw = home.yaw;
			_mission_item.loiter_radius = _navigator->get_loiter_radius();
			_mission_item.acceptance_radius = _navigator->get_acceptance_radius();

			// _mission_item.time_inside = math::max(_param_land_delay.get(), 0.0f);
			// _mission_item.autocontinue = autoland;
			// _mission_item.origin = ORIGIN_ONBOARD;

			// Loiter to deploy landing gear if up
			if ((int)_navigator->get_landing_gear()->landing_gear == (int)landing_gear_s::GEAR_DOWN) {
				// NOTE(Yuneec): If landing gear is down, we don't want to loiter at all.
				// However, loitering for 0 seconds means loitering indefinitely
				// in upstream. Hence we set the time very close to zero:
				_mission_item.time_inside = 2 * FLT_EPSILON;

			} else {
				_mission_item.time_inside = _param_land_delay.get() < 0.0f ? 0.0f : _param_land_delay.get();
			}

			if (autoland && (get_time_inside(_mission_item) > FLT_EPSILON)) {
				_mission_item.nav_cmd = NAV_CMD_LOITER_TIME_LIMIT;
				mavlink_and_console_log_info(_navigator->get_mavlink_log_pub(), "RTL: loiter %.1fs",
							     (double)get_time_inside(_mission_item));

			} else {
				_mission_item.nav_cmd = NAV_CMD_LOITER_UNLIMITED;
				mavlink_and_console_log_info(_navigator->get_mavlink_log_pub(), "RTL: completed, loitering");
			}

			_mission_item.autocontinue = autoland;
			_mission_item.origin = ORIGIN_ONBOARD;
			_mission_item.landing_gear = landing_gear_s::GEAR_DOWN;

			vehicle_command_s cmd_configure{};
			// Set gimbal to
			cmd_configure.command = vehicle_command_s::VEHICLE_CMD_DO_MOUNT_CONFIGURE;
			cmd_configure.param1 = 2.0f; // VEHICLE_MOUNT_MODE_MAVLINK_TARGETING
			cmd_configure.param2 = 0.0f; // don't stabilize roll
			cmd_configure.param3 = 0.0f; // don't stabilize pitch
			cmd_configure.param4 = 0.0f; // don't stabilize yaw (for now this works, later this is done using param7)
			cmd_configure.timestamp = hrt_absolute_time();
			_navigator->publish_vehicle_cmd(&cmd_configure);

			vehicle_command_s cmd_control{};
			// Set gimbal to default orientation
			cmd_control.command = vehicle_command_s::VEHICLE_CMD_DO_MOUNT_CONTROL;
			cmd_control.param1 = 0.0f; // pitch
			cmd_control.param2 = 0.0f; // roll
			cmd_control.param3 = 0.0f; // yaw
			cmd_control.param7 = 2.0f; // VEHICLE_MOUNT_MODE_MAVLINK_TARGETING;
			cmd_control.timestamp = hrt_absolute_time();
			_navigator->publish_vehicle_cmd(&cmd_control);

			_navigator->set_can_loiter_at_sp(true);

			break;
		}

	case RTL_STATE_LAND: {
			// Land at home position.
			_mission_item.nav_cmd = NAV_CMD_LAND;
			_mission_item.lat = home.lat;
			_mission_item.lon = home.lon;
			_mission_item.yaw = home.yaw;
			_mission_item.altitude = home.alt;
			_mission_item.altitude_is_relative = false;
			_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
			_mission_item.time_inside = 0.0f;
			_mission_item.autocontinue = true;
			_mission_item.origin = ORIGIN_ONBOARD;
			_mission_item.landing_gear = landing_gear_s::GEAR_DOWN;

			mavlink_and_console_log_info(_navigator->get_mavlink_log_pub(), "RTL: land at home");
			break;
		}

	case RTL_STATE_LANDED: {
			set_idle_item(&_mission_item);
			set_return_alt_min(false);
			break;
		}

	case RTL_STATE_HOME: {

			_mission_item.lat = home.lat;
			_mission_item.lon = home.lon;
			_mission_item.altitude_is_relative = false;
			_mission_item.altitude = return_alt;
			_mission_item.yaw = NAN;
			_mission_item.loiter_radius = _navigator->get_loiter_radius();
			_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
			_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
			_mission_item.time_inside = 0.0f;
			_mission_item.autocontinue = true;
			_mission_item.origin = ORIGIN_ONBOARD;
			_mission_item.landing_gear = (home_close && home_altitude_close) ? landing_gear_s::GEAR_DOWN : landing_gear_s::GEAR_UP;
			_mission_item.force_velocity = false;

			break;

		}

	default:
		break;
	}

	// it will not set mission yaw setpoint when the vehicle enter fault tolerant control(yaw control tracking is not good)
	if (_navigator->get_esc_report()->motor_state_flags != OK) {
		_mission_item.yaw = NAN;
	}

	reset_mission_item_reached();

	// Execute command if set. This is required for commands like VTOL transition.
	if (!item_contains_position(_mission_item)) {
		issue_command(_mission_item);
	}

	// Convert mission item to current position setpoint and make it valid.
	mission_apply_limitation(_mission_item);

	if (mission_item_to_position_setpoint(_mission_item, &pos_sp_triplet->current)) {
		_navigator->set_position_setpoint_triplet_updated();
	}
}

void
RTL::advance_rtl()
{
	switch (_rtl_state) {

	case RTL_STATE_BRAKE:
		_rtl_state = RTL_STATE_CLIMB;

		break;

	case RTL_STATE_CLIMB:
		_rtl_state = RTL_STATE_PRE_RETURN;

		break;

	case RTL_STATE_PRE_RETURN:
		_rtl_state = RTL_STATE_RETURN;

		break;

	case RTL_STATE_RETURN:
		_rtl_state = RTL_STATE_AFTER_RETURN;

		if (_navigator->get_vstatus()->is_vtol && !_navigator->get_vstatus()->is_rotary_wing) {
			_rtl_state = RTL_STATE_TRANSITION_TO_MC;
		}

		break;

	case RTL_STATE_AFTER_RETURN:
		_rtl_state = RTL_STATE_DESCEND;

		break;

	case RTL_STATE_TRANSITION_TO_MC:
		_rtl_state = RTL_STATE_RETURN;

		break;

	case RTL_STATE_DESCEND:

		// Only go to land if autoland is enabled.
		if (_param_land_delay.get() < -DELAY_SIGMA || _param_land_delay.get() > DELAY_SIGMA) {
			_rtl_state = RTL_STATE_LOITER;

		} else {
			_rtl_state = RTL_STATE_LAND;
		}

		break;

	case RTL_STATE_LOITER:
		_rtl_state = RTL_STATE_LAND;

		break;

	case RTL_STATE_LAND:
		_rtl_state = RTL_STATE_LANDED;

		break;

	case RTL_STATE_HOME:
		_rtl_state = RTL_STATE_DESCEND;

		break;

	default:
		break;
	}
}

float
RTL::get_rtl_altitude()
{

	const float distance_home = get_distance_to_next_waypoint(
					    _return_location.lat,
					    _return_location.lon,
					    _navigator->get_global_position()->lat,
					    _navigator->get_global_position()->lon);

	float climb_alt = _return_location.alt + _param_return_alt.get();

	// if within _param_cone_dist, only climb to a 63.43 (arbitrary) degree cone (two times distance_home)
	if (_param_cone_dist.get() > 0 && distance_home < _param_cone_dist.get()) {
		climb_alt = math::min(
				    _return_location.alt + 2.0f * distance_home,
				    climb_alt);
	}

	// do also not reduce altitude if already higher
	climb_alt = math::max(climb_alt, _navigator->get_global_position()->alt);

	// and also make sure that an absolute minimum altitude is obeyed so the landing gear does not catch.
	climb_alt = math::max(climb_alt,
			      _return_location.alt + _param_min_loiter_alt.get());

	// if RTL altitude is greater than the maximum allowed altitude, change RTL climb altitude to be inside the geofence
	if ((_param_return_alt.get() > _param_gf_alt.get()
	     && _param_gf_alt.get() > FLT_EPSILON)
	    && ((uint8_t) _param_gf_actions.get()
		!= geofence_result_s::GF_ACTION_NONE)) {
		climb_alt = math::min(climb_alt,
				      _return_location.alt + _param_gf_alt.get());
		/* This is a safety check to stay higher than people's heads. */
		float safe_altitude = 5.0f;
		climb_alt = math::max(climb_alt,
				      _return_location.alt + safe_altitude);
	}

	return climb_alt;
}

void
RTL::set_GCS_to_home(home_position_s &hpos, const vehicle_global_position_s &pos, const follow_target_s &target)
{
	// keep a safe distance to GCS
	constexpr float safe_distance = 5.0f;

	// get bearing from GCS to current target
	const float bearing = get_bearing_to_next_waypoint(target.lat, target.lon, pos.lat, pos.lon);

	// set home position a safe distance away but on the flight path
	waypoint_from_heading_and_distance(target.lat, target.lon,
					   bearing, safe_distance,
					   &hpos.lat, &hpos.lon);
}

void
RTL::update_return_location()
{
	// default home is where takeoff location was, home altitude is used in any case
	_return_location = *_navigator->get_home_position();

	if (_param_home_at_gcs.get()) {
		const follow_target_s &target = *_navigator->get_target_motion();

		// replace only horizontal landing position if target is finite, altitude stays from home
		if (PX4_ISFINITE(target.lat) && PX4_ISFINITE(target.lon)) {
			const vehicle_global_position_s &gpos = *_navigator->get_global_position();
			set_GCS_to_home(_return_location, gpos, target);
		}
	}
}

void
RTL::calc_and_pub_rtl_time_estimate()
{
	// Advertise uORB topic the first time
	if (_rtl_time_estimate_pub == nullptr) {
		_rtl_time_estimate_pub = orb_advertise(ORB_ID(rtl_time_estimate),
						       &_rtl_time_estimate);
		return;
	}

	_rtl_time_estimate.time_estimate = 0;
	_rtl_time_estimate.safe_time_estimate = 0;

	// Calculate RTL time estimate only when there is a valid home position
	// TODO: Also check if vehicle position is valid
	if (!_navigator->home_position_valid()) {
		_rtl_time_estimate.valid = false;

	} else {
		_rtl_time_estimate.valid = true;

		const vehicle_global_position_s &gpos = *_navigator->get_global_position();

		// compute the return altitude. This takes the yuneec-cone into account
		float return_alt;

		if (_rtl_state == RTL_STATE_NONE) {
			// While RTL is inactive, calculate the RTL altitude the same way we would
			// if RTL was activated right here and now.
			return_alt = get_rtl_altitude();

		} else {
			// RTL is currently active. Get return altitude directly from RTL
			return_alt = _mission_item.altitude;
		}

		// Sum up time estimate for various segments of the landing procedure
		switch (_rtl_state) {
		case RTL_STATE_NONE:

		// Fallthrough intented
		case RTL_STATE_BRAKE:

		// Fallthrough intented
		case RTL_STATE_CLIMB: {
				// Climb segment is only relevant if the drone is below return altitude
				const float climb_dist = gpos.alt < return_alt ? (return_alt - gpos.alt) : 0;

				if (climb_dist > 0) {
					_rtl_time_estimate.time_estimate += climb_dist / _param_mpc_vel_z_auto.get();
				}
			}

		// Fallthrough intented
		case RTL_STATE_PRE_RETURN:

		// Fallthrough intented
		case RTL_STATE_RETURN:

			// Add cruise segment to home
			_rtl_time_estimate.time_estimate += get_distance_to_next_waypoint(
					_return_location.lat, _return_location.lon,	gpos.lat, gpos.lon) / _param_mpc_xy_cruise.get();

		// Fallthrough intented
		case RTL_STATE_AFTER_RETURN:

		// Fallthrough intented
		case RTL_STATE_TRANSITION_TO_MC:

		// Fallthrough intented
		case RTL_STATE_DESCEND: {
				// when descending, the target altitude is stored in the current mission item
				float initial_altitude = 0;
				float loiter_altitude = 0;

				if (_rtl_state == RTL_STATE_DESCEND) {
					// Take current vehicle altitude as the starting point for calculation
					initial_altitude = gpos.alt;  // TODO: Check if this is in the right frame
					loiter_altitude = _mission_item.altitude;  // Next waypoint = loiter


				} else {
					// Take the return altitude as the starting point for the calculation
					initial_altitude = return_alt; // CLIMB and RETURN
					loiter_altitude = math::min(_return_location.alt + _param_descend_alt.get(), return_alt);
				}

				// Add descend segment (first landing phase: return alt to loiter alt)
				_rtl_time_estimate.time_estimate += fabsf(initial_altitude - loiter_altitude) /
								    _param_mpc_vel_z_auto.get();
			}

		// Fallthrough intented
		case RTL_STATE_LOITER:
			// Add land delay (the short pause for deploying landing gear)
			// TODO: Check if landing gear is deployed or not
			_rtl_time_estimate.time_estimate += _param_land_delay.get();

		case RTL_STATE_LAND: {
				float initial_altitude;

				// Add land segment (second landing phase) which comes after LOITER
				if (_rtl_state == RTL_STATE_LAND) {
					// If we are in this phase, use the current vehicle altitude  instead
					// of the altitude paramteter to get a continous time estimate
					initial_altitude = gpos.alt;


				} else {
					// If this phase is not active yet, simply use the loiter altitude,
					// which is where the LAND phase will start
					const float loiter_altitude = math::min(_return_location.alt + _param_descend_alt.get(), return_alt);
					initial_altitude = loiter_altitude;
				}

				// Prevent negative times when close to the ground
				if (initial_altitude > _return_location.alt) {
					_rtl_time_estimate.time_estimate += (initial_altitude - _return_location.alt) / _param_mpc_land_speed.get();
				}

			}

			break;

		case RTL_STATE_LANDED:
			// Remaining time is 0
			break;

		case RTL_STATE_HOME:
			// Remaining time is 0
			break;

		default:
			break;
		}

		// Prevent negative durations as phyiscally they make no sense. These can
		// occur during the last phase of landing when close to the ground.
		if (_rtl_time_estimate.time_estimate < 0) {
			_rtl_time_estimate.time_estimate = 0;
		}

		// Use actual time estimate to compute the safer time estimate with additional
		// scale factor and a margin
		_rtl_time_estimate.safe_time_estimate =
			_param_rtl_time_factor.get() * _rtl_time_estimate.time_estimate +
			_param_rtl_time_margin.get();
	}

	// Publish message
	_rtl_time_estimate.timestamp = hrt_absolute_time();
	orb_publish(ORB_ID(rtl_time_estimate), _rtl_time_estimate_pub, &_rtl_time_estimate);
}
