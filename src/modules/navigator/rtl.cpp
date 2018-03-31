/****************************************************************************
 *
 *   Copyright (c) 2013-2017 PX4 Development Team. All rights reserved.
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
 * @author Lorenz Meier <lorenz@px4.io>
 */

#include "rtl.h"
#include "navigator.h"

#include <cfloat>

#include <mathlib/mathlib.h>
#include <systemlib/mavlink_log.h>
#include <uORB/topics/vehicle_command.h>

using math::max;
using math::min;

static constexpr float DELAY_SIGMA = 0.01f;

RTL::RTL(Navigator *navigator, const char *name) :
	MissionBlock(navigator, name),
	_param_return_alt(this, "RTL_RETURN_ALT", false),
	_param_min_loiter_alt(this, "MIS_LTRMIN_ALT", false),  // NOTE: Yuneec specific
	_param_descend_alt(this, "RTL_DESCEND_ALT", false),
	_param_land_delay(this, "RTL_LAND_DELAY", false),
	_param_rtl_min_dist(this, "RTL_MIN_DIST", false),
	_param_rtl_land_type(this, "RTL_LAND_TYPE", false),
	_param_gf_alt(this, "GF_MAX_VER_DIST", false),
	_param_gf_actions(this, "GF_ACTION", false)
{
}

void
RTL::on_inactive()
{
	// reset RTL state
	_rtl_state = RTL_STATE_NONE;
}

bool
RTL::mission_landing_required()
{
	// returns true if navigator should use planned mission landing
	return (_param_rtl_land_type.get() == 1);
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

	// go directly to home if obstacle avoidance is used
	const bool obstacle_avoidance_switch_on = _navigator->get_manual_setpoint()->obsavoid_switch ==
			manual_control_setpoint_s::SWITCH_POS_ON;
	const bool obstacle_avoidance_running = _navigator->get_trajectory_waypoint()->point_valid[0] == true;

	/* for safety reasons don't go into RTL if landed */
	if (_navigator->get_land_detected()->landed) {
		// for safety reasons don't go into RTL if landed
		_rtl_state = RTL_STATE_LANDED;
		mavlink_log_info(_navigator->get_mavlink_log_pub(), "Already landed, not executing RTL");
		// otherwise start RTL by braking first

	} else if (obstacle_avoidance_running && obstacle_avoidance_switch_on) {
		_rtl_state = RTL_STATE_HOME;
		mavlink_log_info(_navigator->get_mavlink_log_pub(), "Flying straight to home");

	} else if (mission_landing_required() && _navigator->on_mission_landing()) {
		// RTL straight to RETURN state, but mission will takeover for landing
		_rtl_state = RTL_STATE_RETURN;

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
	if (_rtl_state != RTL_STATE_LANDED && is_mission_item_reached()) {
		advance_rtl();
		set_rtl_item();
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
	// RTL_TYPE: mission landing
	// landing using planned mission landing, fly to DO_LAND_START instead of returning HOME
	// do nothing, let navigator takeover with mission landing
	if (mission_landing_required()) {
		if (_rtl_state > RTL_STATE_CLIMB) {
			if (_navigator->start_mission_landing()) {
				mavlink_and_console_log_info(_navigator->get_mavlink_log_pub(), "RTL: using mission landing");
				return;

			} else {
				// otherwise use regular RTL
				mavlink_log_critical(_navigator->get_mavlink_log_pub(), "RTL: unable to use mission landing");
			}
		}
	}

	_navigator->set_can_loiter_at_sp(false);

	const home_position_s &home = *_navigator->get_home_position();
	const vehicle_global_position_s &gpos = *_navigator->get_global_position();

	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	// use home yaw if close to home
	float home_dist = get_distance_to_next_waypoint(_navigator->get_home_position()->lat,
			  _navigator->get_home_position()->lon,
			  _navigator->get_global_position()->lat, _navigator->get_global_position()->lon);

	bool home_close = (home_dist < _param_rtl_min_dist.get());
	bool home_altitude_close = (fabsf(_navigator->get_home_position()->alt - _navigator->get_global_position()->alt) <
				    _param_rtl_min_dist.get());

	switch (_rtl_state) {
	case RTL_STATE_BRAKE: {

			/* slow down before climbing */
			set_brake_item(&_mission_item);

			break;

		}

	case RTL_STATE_CLIMB: {

			_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
			_mission_item.lat = gpos.lat;
			_mission_item.lon = gpos.lon;
			_mission_item.altitude_is_relative = false;
			_mission_item.altitude = get_rtl_altitude();
			_mission_item.yaw = NAN;
			_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
			_mission_item.time_inside = 0.0f;
			_mission_item.autocontinue = true;
			_mission_item.origin = ORIGIN_ONBOARD;
			_mission_item.deploy_gear = home_close && home_altitude_close;
			_mission_item.force_velocity = false;

			mavlink_and_console_log_info(_navigator->get_mavlink_log_pub(), "RTL: climb to %d m (%d m above home)",
						     (int)ceilf(_mission_item.altitude), (int)ceilf(_mission_item.altitude - _navigator->get_home_position()->alt));
			break;
		}

	case RTL_STATE_PRE_RETURN: {

			_mission_item.lat = gpos.lat;
			_mission_item.lon = gpos.lon;

			if (home_close) {
				_mission_item.yaw = _navigator->get_home_position()->yaw;

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
			_mission_item.deploy_gear = home_close && home_altitude_close;
			break;
		}

	case RTL_STATE_RETURN: {

			// don't change altitude
			_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
			_mission_item.lat = home.lat;
			_mission_item.lon = home.lon;
			_mission_item.altitude = gpos.alt;
			_mission_item.altitude_is_relative = false;

			// use home yaw if close to home
			if (home_close) {
				_mission_item.yaw = home.yaw;

			} else {
				// use current heading to home
				_mission_item.yaw = get_bearing_to_next_waypoint(gpos.lat, gpos.lon, home.lat, home.lon);
			}

			_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
			_mission_item.time_inside = 0.0f;
			_mission_item.autocontinue = true;
			_mission_item.origin = ORIGIN_ONBOARD;
			_mission_item.deploy_gear = home_close && home_altitude_close;

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
			_mission_item.deploy_gear = home_close && home_altitude_close;
			break;
		}

	case RTL_STATE_TRANSITION_TO_MC: {
			_mission_item.nav_cmd = NAV_CMD_DO_VTOL_TRANSITION;
			_mission_item.params[0] = vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC;
			break;
		}

	case RTL_STATE_DESCEND: {
			_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
			_mission_item.lat = home.lat;
			_mission_item.lon = home.lon;
			_mission_item.altitude = min(home.alt + _param_descend_alt.get(), gpos.alt);
			_mission_item.altitude_is_relative = false;

			// except for vtol which might be still off here and should point towards this location
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
			_mission_item.deploy_gear = home_close && home_altitude_close;

			/* disable previous setpoint to prevent drift */
			pos_sp_triplet->previous.valid = false;
			break;
		}

	case RTL_STATE_LOITER: {
			const bool autoland = (_param_land_delay.get() > FLT_EPSILON);

			// don't change altitude
			_mission_item.lat = home.lat;
			_mission_item.lon = home.lon;
			_mission_item.altitude = gpos.alt;
			_mission_item.altitude_is_relative = false;
			_mission_item.yaw = home.yaw;
			_mission_item.loiter_radius = _navigator->get_loiter_radius();
			_mission_item.acceptance_radius = _navigator->get_acceptance_radius();

			// Loiter to deploy landing gear if up
			if ((int)_navigator->get_attitude_sp()->landing_gear == (int)vehicle_attitude_setpoint_s::LANDING_GEAR_DOWN) {
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
			_mission_item.deploy_gear = true;

			vehicle_command_s cmd{};
			// Set gimbal to default orientation
			cmd.command = vehicle_command_s::VEHICLE_CMD_DO_MOUNT_CONTROL;
			cmd.param1 = 0.0f; // pitch
			cmd.param2 = 0.0f; // roll
			cmd.param3 = 0.0f; // yaw
			cmd.param7 = 2.0f; // VEHICLE_MOUNT_MODE_MAVLINK_TARGETING;
			cmd.timestamp = hrt_absolute_time();
			_navigator->publish_vehicle_cmd(&cmd);

			_navigator->set_can_loiter_at_sp(true);

			break;
		}

	case RTL_STATE_LAND: {
			// land at home position
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
			_mission_item.deploy_gear = true;

			mavlink_log_critical(_navigator->get_mavlink_log_pub(), "RTL: land at home");
			break;
		}

	case RTL_STATE_LANDED: {
			set_idle_item(&_mission_item);
			set_return_alt_min(false);
			break;
		}

	case RTL_STATE_HOME: {

			_mission_item.lat = _navigator->get_home_position()->lat;
			_mission_item.lon = _navigator->get_home_position()->lon;
			_mission_item.altitude_is_relative = false;
			_mission_item.altitude = get_rtl_altitude();
			_mission_item.yaw = NAN;
			_mission_item.loiter_radius = _navigator->get_loiter_radius();
			_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
			_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
			_mission_item.time_inside = 0.0f;
			_mission_item.autocontinue = true;
			_mission_item.origin = ORIGIN_ONBOARD;
			_mission_item.deploy_gear = home_close && home_altitude_close;
			_mission_item.force_velocity = false;

			break;

		}

	default:
		break;
	}

	// it will not set mission yaw setpoint when the vehicle enter fault tolerant control(yaw control tracking is not good)
	if (_navigator->get_esc_report()->engine_failure_report.motor_state != OK) {
		_mission_item.yaw = NAN;

	}

	reset_mission_item_reached();

	/* execute command if set. This is required for commands like VTOL transition */
	if (!item_contains_position(_mission_item)) {
		issue_command(_mission_item);
	}

	/* convert mission item to current position setpoint and make it valid */
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

		/* only go to land if autoland is enabled */
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

	// climb to at least a 45 degree cone
	float climb_alt = _navigator->get_home_position()->alt
			  + get_distance_to_next_waypoint(
				  _navigator->get_home_position()->lat,
				  _navigator->get_home_position()->lon,
				  _navigator->get_global_position()->lat,
				  _navigator->get_global_position()->lon);

	/* limit altitude to rtl max */
	climb_alt = math::min(
			    _navigator->get_home_position()->alt + _param_return_alt.get(),
			    climb_alt);
	// do also not reduce altitude if already higher
	climb_alt = math::max(climb_alt, _navigator->get_global_position()->alt);

	// and also make sure that an absolute minimum altitude is obeyed so the landing gear does not catch.
	climb_alt = math::max(climb_alt,
			      _navigator->get_home_position()->alt + _param_min_loiter_alt.get());

	// if RTL altitude is greater than the maximum allowed altitude, change RTL climb altitude to be inside the geofence
	if ((_param_return_alt.get() > _param_gf_alt.get()
	     && _param_gf_alt.get() > FLT_EPSILON)
	    && ((uint8_t) _param_gf_actions.get()
		!= geofence_result_s::GF_ACTION_NONE)) {
		climb_alt = math::min(climb_alt,
				      _navigator->get_home_position()->alt + _param_gf_alt.get());
		/* This is a safety check to stay higher than people's heads. */
		float safe_altitude = 5.0f;
		climb_alt = math::max(climb_alt,
				      _navigator->get_home_position()->alt + safe_altitude);
	}

	return climb_alt;

}
