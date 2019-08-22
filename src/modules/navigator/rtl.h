/***************************************************************************
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
 * @file rtl.h
 *
 * Helper class for RTL
 *
 * @author Julian Oes <julian@oes.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 * @author Lorenz Meier <lorenz@px4.io>
 */

#pragma once

#include <px4_module_params.h>

#include <uORB/topics/geofence_result.h>
#include "uORB/topics/home_position.h"
#include <uORB/topics/follow_target.h>
#include <uORB/topics/rtl_time_estimate.h>

#include "navigator_mode.h"
#include "mission_block.h"

class Navigator;

class RTL : public MissionBlock, public ModuleParams
{
public:
	enum RTLType {
		RTL_HOME = 0,
		RTL_LAND,
		RTL_MISSION,
	};

	RTL(Navigator *navigator);

	~RTL();

	void on_inactive() override;
	void on_activation() override;
	void on_active() override;

	void set_return_alt_min(bool min);

	int rtl_type() const;

private:
	/**
	 * Set the RTL item
	 */
	void		set_rtl_item();

	/**
	 * Move to next RTL item
	 */
	void		advance_rtl();

	/*
	 * Get rtl altitude
	 * NOTE: Yuneec specific function. A function with the same name existed on
	 * Upstream before (now deleted), but we made modifications to it. See
	 * https://github.com/PX4/Firmware/commit/84f07c64b0745da86418083252dd1ec732e3e4df
	 */
	float 		get_rtl_altitude();

	/**
	 * Set return location.
	 * @param home_position that gets set to ground control station
	 * @param pos that represents the vehicle current pose
	 */
	void set_GCS_to_home(home_position_s &home_position, const vehicle_global_position_s &pos,
			     const follow_target_s &target);

	/**
	 * Compute and publish the estimated time to perform RTL from the current location.
	 */
	void		calc_and_pub_rtl_time_estimate();

	/**
	 * Set return location to either the launch-site or current position of the
	 * GroundStation, depending on the paramter settings.
	 */
	void update_return_location();

	/**
	 * Return location.
	 * The location can be the takeoff position or the position
	 * of the remote controller.
	 */
	home_position_s _return_location{};

	enum RTLState {
		RTL_STATE_NONE = 0,
		RTL_STATE_BRAKE,
		RTL_STATE_CLIMB,
		RTL_STATE_RETURN,
		RTL_STATE_TRANSITION_TO_MC,
		RTL_STATE_DESCEND,
		RTL_STATE_LOITER,
		RTL_STATE_LAND,
		RTL_STATE_LANDED,
		RTL_STATE_PRE_RETURN,
		RTL_STATE_AFTER_RETURN,
		RTL_STATE_HOME,
	} _rtl_state{RTL_STATE_NONE};

	bool _rtl_alt_min{false};
	orb_advert_t _rtl_time_estimate_pub = nullptr;
	rtl_time_estimate_s _rtl_time_estimate {};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::RTL_RETURN_ALT>) _param_return_alt,
		(ParamFloat<px4::params::MIS_TAKEOFF_ALT>) _param_min_takeoff_alt, // NOTE: Yuneec specific, has been deleted upstream
		(ParamFloat<px4::params::GF_MAX_VER_DIST>) _param_gf_alt,
		(ParamInt<px4::params::GF_ACTION>) _param_gf_actions,
		(ParamFloat<px4::params::RTL_DESCEND_ALT>) _param_descend_alt,
		(ParamFloat<px4::params::RTL_LAND_DELAY>) _param_land_delay,
		(ParamFloat<px4::params::RTL_MIN_DIST>) _param_rtl_min_dist,
		(ParamInt<px4::params::RTL_TYPE>) _param_rtl_type,
		(ParamInt<px4::params::RTL_TO_GCS>) _param_home_at_gcs, // home position is where GCS is located
		(ParamFloat<px4::params::RTL_CONE_DIST>) _param_cone_dist,
		(ParamFloat<px4::params::MPC_VEL_Z_AUTO>) _param_mpc_vel_z_auto,
		(ParamFloat<px4::params::MPC_XY_CRUISE>) _param_mpc_xy_cruise,
		(ParamFloat<px4::params::MPC_LAND_SPEED>) _param_mpc_land_speed,
		(ParamFloat<px4::params::RTL_TIME_FACTOR>) _param_rtl_time_factor,
		(ParamInt<px4::params::RTL_TIME_MARGIN>) _param_rtl_time_margin
	)
};
