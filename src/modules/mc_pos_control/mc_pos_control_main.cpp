/****************************************************************************
 *
 *   Copyright (c) 2013 - 2017 PX4 Development Team. All rights reserved.
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
 * @file mc_pos_control_main.cpp
 * Multicopter position controller.
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_module_params.h>
#include <px4_tasks.h>
#include <px4_module.h>
#include <px4_posix.h>
#include <drivers/drv_hrt.h>
#include <lib/hysteresis/hysteresis.h>
#include <commander/px4_custom_mode.h>

#include <uORB/topics/home_position.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_trajectory_waypoint.h>
#include <uORB/topics/landing_gear.h>

#include <float.h>
#include <mathlib/mathlib.h>
#include <systemlib/mavlink_log.h>

#include <controllib/blocks.hpp>

#include <lib/FlightTasks/FlightTasks.hpp>
#include <lib/WeatherVane/WeatherVane.hpp>
#include "PositionControl.hpp"
#include "Utility/ControlMath.hpp"
#include "Takeoff.hpp"

/* --- yuneec specific */
#include <uORB/topics/landing_gear.h>
/* --- END yuneec specific */

using namespace time_literals;

/**
 * Multicopter position control app start / stop handling function
 */
extern "C" __EXPORT int mc_pos_control_main(int argc, char *argv[]);

class MulticopterPositionControl : public ModuleBase<MulticopterPositionControl>, public control::SuperBlock,
	public ModuleParams
{
public:
	MulticopterPositionControl();

	~MulticopterPositionControl();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static MulticopterPositionControl *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

private:
	Takeoff _takeoff; /**< state machine and ramp to bring the vehicle off the ground without jumps */

	/* --- yuneec specific declarations */
	/* landing gear */
	int		_manual_sub{-1};			/**< notification of manual control updates */
	struct manual_control_setpoint_s		_manual = {};		/**< r/c channel data */
	/* --- END yuneec specific declarations */

	bool 		_in_smooth_takeoff = false; 		/**<true if takeoff ramp is applied */

	orb_advert_t	_att_sp_pub{nullptr};			/**< attitude setpoint publication */
	orb_advert_t	_traj_sp_pub{nullptr};		/**< trajectory setpoints publication */
	orb_advert_t	_local_pos_sp_pub{nullptr};		/**< vehicle local position setpoint publication */
	orb_advert_t 	_pub_vehicle_command{nullptr};           /**< vehicle command publication */
	orb_advert_t _mavlink_log_pub{nullptr};

	orb_advert_t 	_flighttask_sp_pub{nullptr}; /**< flighttask setpoint publication for logging (only for logging)*/
	orb_advert_t	_position_control_setpoint_input_pub{nullptr}; /** setpoint input to position controller (only for logging)*/
	orb_advert_t	_traj_wp_avoidance_desired_pub{nullptr}; /**< trajectory waypoint desired publication */
	orb_advert_t 	_constraints_pub{nullptr}; /**< vehicle constraints publication (only for logging)*/
	orb_id_t _attitude_setpoint_id{nullptr};
	orb_advert_t	_landing_gear_pub{nullptr};

	int		_vehicle_status_sub{-1};		/**< vehicle status subscription */
	int		_vehicle_land_detected_sub{-1};	/**< vehicle land detected subscription */
	int		_control_mode_sub{-1};		/**< vehicle control mode subscription */
	int		_params_sub{-1};			/**< notification of parameter updates */
	int		_local_pos_sub{-1};			/**< vehicle local position */
	int		_att_sub{-1};				/**< vehicle attitude */
	int		_home_pos_sub{-1}; 			/**< home position */
	int		_traj_wp_avoidance_sub{-1};	/**< trajectory waypoint */

	int _task_failure_count{0};         /**< counter for task failures */

	vehicle_status_s 			_vehicle_status{};		/**< vehicle status */
	/**< vehicle-land-detection: initialze to landed */
	vehicle_land_detected_s _vehicle_land_detected = {
		.timestamp = 0,
		.alt_max = -1.0f,
		.landed = true,
		.freefall = false,
		.ground_contact = false,
		.maybe_landed = false,
		.crash = false,
		.inverted = false,
		._padding0 = {}
	};

	vehicle_attitude_setpoint_s	_att_sp{};			/**< vehicle attitude setpoint */
	vehicle_control_mode_s	_control_mode{};		/**< vehicle control mode */
	vehicle_local_position_s _local_pos{};			/**< vehicle local position */
	home_position_s	_home_pos{};			/**< home position */
	vehicle_trajectory_waypoint_s _traj_wp_avoidance{};		/**< trajectory waypoint */
	vehicle_trajectory_waypoint_s _traj_wp_avoidance_desired{};	/**< desired waypoints, inputs to an obstacle avoidance module */
	landing_gear_s _landing_gear{};
	int8_t _old_landing_gear_position{landing_gear_s::GEAR_KEEP};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::MPC_TKO_RAMP_T>) _param_mpc_tko_ramp_t, /**< time constant for smooth takeoff ramp */
		(ParamFloat<px4::params::MPC_XY_VEL_MAX>) _param_mpc_xy_vel_max,
		(ParamFloat<px4::params::MPC_VEL_MANUAL>) _param_mpc_vel_manual,
		(ParamFloat<px4::params::MPC_XY_CRUISE>) _param_mpc_xy_cruise,
		(ParamFloat<px4::params::MPC_Z_VEL_MAX_UP>) _param_mpc_z_vel_max_up,
		(ParamFloat<px4::params::MPC_Z_VEL_MAX_DN>) _param_mpc_z_vel_max_dn,
		(ParamFloat<px4::params::MPC_LAND_SPEED>) _param_mpc_land_speed,
		(ParamFloat<px4::params::MPC_TKO_SPEED>) _param_mpc_tko_speed,
		(ParamFloat<px4::params::MPC_LAND_ALT2>) _param_mpc_land_alt2, /**< downwards speed limited below this altitude */
		(ParamInt<px4::params::MPC_POS_MODE>) _param_mpc_pos_mode,
		(ParamInt<px4::params::MPC_AUTO_MODE>) _param_mpc_auto_mode,
		(ParamInt<px4::params::MPC_ALT_MODE>) _param_mpc_alt_mode,
		(ParamFloat<px4::params::MPC_SPOOLUP_TIME>) _param_mpc_spoolup_time, /**< time to let motors spool up after arming */
		(ParamFloat<px4::params::MPC_TILTMAX_LND>) _param_mpc_tiltmax_lnd, /**< maximum tilt for landing and smooth takeoff */
		(ParamFloat<px4::params::MPC_THR_MIN>)_param_mpc_thr_min,
		(ParamFloat<px4::params::MPC_THR_HOVER>)_param_mpc_thr_hover,
		(ParamFloat<px4::params::MPC_THR_MAX>)_param_mpc_thr_max,
		(ParamFloat<px4::params::MPC_Z_VEL_P>)_param_mpc_z_vel_p
	);

	control::BlockDerivative _vel_x_deriv; /**< velocity derivative in x */
	control::BlockDerivative _vel_y_deriv; /**< velocity derivative in y */
	control::BlockDerivative _vel_z_deriv; /**< velocity derivative in z */

	FlightTasks _flight_tasks; /**< class generating position controller setpoints depending on vehicle task */
	PositionControl _control; /**< class for core PID position control */
	PositionControlStates _states{}; /**< structure containing vehicle state information for position control */

	hrt_abstime _last_warn = 0; /**< timer when the last warn message was sent out */

	bool _in_failsafe = false; /**< true if failsafe was entered within current cycle */

	/** Timeout in us for trajectory data to get considered invalid */
	static constexpr uint64_t TRAJECTORY_STREAM_TIMEOUT_US = 500_ms;
	/** number of tries before switching to a failsafe flight task */
	static constexpr int NUM_FAILURE_TRIES = 10;
	/** If Flighttask fails, keep 0.2 seconds the current setpoint before going into failsafe land */
	static constexpr uint64_t LOITER_TIME_BEFORE_DESCEND = 200_ms;
	/** During smooth-takeoff, below ALTITUDE_THRESHOLD the yaw-control is turned off ant tilt is limited */
	static constexpr float ALTITUDE_THRESHOLD = 0.3f;

	systemlib::Hysteresis _failsafe_land_hysteresis{false}; /**< becomes true if task did not update correctly for LOITER_TIME_BEFORE_DESCEND */

	WeatherVane *_wv_controller{nullptr};

	/**
	 * Update our local parameter cache.
	 * Parameter update can be forced when argument is true.
	 * @param force forces parameter update.
	 */
	int parameters_update(bool force);

	/**
	 * Check for changes in subscribed topics.
	 */
	void poll_subscriptions();

	/**
	 * Check for validity of positon/velocity states.
	 * @param vel_sp_z velocity setpoint in z-direction
	 */
	void set_vehicle_states(const float &vel_sp_z);

	/**
	 * Limit altitude based on land-detector.
	 * @param setpoint needed to detect vehicle intention.
	 */
	void limit_altitude(vehicle_local_position_setpoint_s &setpoint);

	/**
	 * Prints a warning message at a lowered rate.
	 * @param str the message that has to be printed.
	 */
	void warn_rate_limited(const char *str);

	/**
	 * Publish attitude.
	 */
	void publish_attitude();

	/**
	 * Publish local position setpoint.
	 * This is only required for logging.
	 */
	void publish_local_pos_sp(const vehicle_local_position_setpoint_s &local_pos_sp);

	/**
	 * Publish local position setpoint.
	 * This is only required for logging.
	 */
	void publish_trajectory_sp(const vehicle_local_position_setpoint_s &traj);

	/**
	 * Adjust the setpoint during landing.
	 * Thrust is adjusted to support the land-detector during detection.
	 * @param setpoint gets adjusted based on land-detector state
	 */
	void limit_thrust_during_landing(vehicle_local_position_setpoint_s &setpoint);

	/**
	 * Start flightasks based on navigation state.
	 * This methods activates a task based on the navigation state.
	 */
	void start_flight_task();

	/**
	 * Failsafe.
	 * If flighttask fails for whatever reason, then do failsafe. This could
	 * occur if the commander fails to switch to a mode in case of invalid states or
	 * setpoints. The failsafe will occur after LOITER_TIME_BEFORE_DESCEND. If force is set
	 * to true, the failsafe will be initiated immediately.
	 */
	void failsafe(vehicle_local_position_setpoint_s &setpoint, const PositionControlStates &states, const bool force,
		      const bool warn);

	/**
	 * Fill desired vehicle_trajectory_waypoint:
	 * point1: current position, desired velocity
	 * point2: current triplet only if in auto mode
	 * @param states current vehicle state
	 */
	void update_avoidance_waypoint_desired(PositionControlStates &states, vehicle_local_position_setpoint_s &setpoint);

	/**
	 * Check whether or not use the obstacle avoidance waypoint
	 */
	bool use_obstacle_avoidance();

	/**
	 * Reset setpoints to NAN
	 */
	void reset_setpoint_to_nan(vehicle_local_position_setpoint_s &setpoint);

	/**
	 * Overwrite setpoint with waypoint coming from obstacle avoidance
	 */
	void execute_avoidance_waypoint(vehicle_local_position_setpoint_s &setpoint);

	/**
	 * Publish desired vehicle_trajectory_waypoint
	 */
	void publish_avoidance_desired_waypoint();

	/**
	 * Shim for calling task_main from task_create.
	 */
	static int	task_main_trampoline(int argc, char *argv[]);

	/**
	 * check if task should be switched because of failsafe
	 */
	void check_failure(bool task_failure, uint8_t nav_state);

	/**
	 * send vehicle command to inform commander about failsafe
	 */
	void send_vehicle_cmd_do(uint8_t nav_state);

	/**
	 * Main sensor collection task.
	 */
	void		task_main();
};


int MulticopterPositionControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
The controller has two loops: a P loop for position error and a PID loop for velocity error.
Output of the velocity controller is thrust vector that is split to thrust direction
(i.e. rotation matrix for multicopter orientation) and thrust scalar (i.e. multicopter thrust itself).

The controller doesn't use Euler angles for its work, they are generated only for more human-friendly control and
logging.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mc_pos_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}


MulticopterPositionControl::MulticopterPositionControl() :
	SuperBlock(nullptr, "MPC"),
	ModuleParams(nullptr),
	_vel_x_deriv(this, "VELD"),
	_vel_y_deriv(this, "VELD"),
	_vel_z_deriv(this, "VELD"),
	_control(this)
{
	// fetch initial parameter values
	parameters_update(true);
	// set failsafe hysteresis
	_failsafe_land_hysteresis.set_hysteresis_time_from(false, LOITER_TIME_BEFORE_DESCEND);
}

MulticopterPositionControl::~MulticopterPositionControl()
{
	if (_wv_controller != nullptr) {
		delete _wv_controller;
	}
}

void
MulticopterPositionControl::warn_rate_limited(const char *string)
{
	hrt_abstime now = hrt_absolute_time();

	if (now - _last_warn > 200_ms) {
		PX4_WARN("%s", string);
		_last_warn = now;
	}
}

int
MulticopterPositionControl::parameters_update(bool force)
{
	bool updated;
	struct parameter_update_s param_upd;

	orb_check(_params_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(parameter_update), _params_sub, &param_upd);
	}

	if (updated || force) {
		ModuleParams::updateParams();
		SuperBlock::updateParams();

		// Check that the design parameters are inside the absolute maximum constraints
		if (_param_mpc_xy_cruise.get() > _param_mpc_xy_vel_max.get()) {
			_param_mpc_xy_cruise.set(_param_mpc_xy_vel_max.get());
			_param_mpc_xy_cruise.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Cruise speed has been constrained by max speed")
		}

		if (_param_mpc_vel_manual.get() > _param_mpc_xy_vel_max.get()) {
			_param_mpc_vel_manual.set(_param_mpc_xy_vel_max.get());
			_param_mpc_vel_manual.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Manual speed has been constrained by max speed")
		}

		if (_param_mpc_thr_hover.get() > _param_mpc_thr_max.get() ||
		    _param_mpc_thr_hover.get() < _param_mpc_thr_min.get()) {
			_param_mpc_thr_hover.set(math::constrain(_param_mpc_thr_hover.get(), _param_mpc_thr_min.get(),
						 _param_mpc_thr_max.get()));
			_param_mpc_thr_hover.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Hover thrust has been constrained by min/max")
		}

		_flight_tasks.handleParameterUpdate();

		// initialize vectors from params and enforce constraints
		_param_mpc_tko_speed.set(math::min(_param_mpc_tko_speed.get(), _param_mpc_z_vel_max_up.get()));
		_param_mpc_land_speed.set(math::min(_param_mpc_land_speed.get(), _param_mpc_z_vel_max_dn.get()));

		// set trigger time for takeoff delay
		_takeoff.setSpoolupTime(_param_mpc_spoolup_time.get());
		_takeoff.setTakeoffRampTime(_param_mpc_tko_ramp_t.get());
		_takeoff.generateInitialRampValue(_param_mpc_thr_hover.get(), _param_mpc_z_vel_p.get());

		if (_wv_controller != nullptr) {
			_wv_controller->update_parameters();
		}
	}

	return OK;
}

void
MulticopterPositionControl::poll_subscriptions()
{
	// This is polled for, so all we need to do is a copy now.
	orb_copy(ORB_ID(vehicle_local_position), _local_pos_sub, &_local_pos);

	bool updated;

	orb_check(_vehicle_status_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &_vehicle_status);

		// set correct uORB ID, depending on if vehicle is VTOL or not
		if (!_attitude_setpoint_id) {
			if (_vehicle_status.is_vtol) {
				_attitude_setpoint_id = ORB_ID(mc_virtual_attitude_setpoint);

			} else {
				_attitude_setpoint_id = ORB_ID(vehicle_attitude_setpoint);
			}
		}

		// if vehicle is a VTOL we want to enable weathervane capabilities
		if (_wv_controller == nullptr && _vehicle_status.is_vtol) {
			_wv_controller = new WeatherVane();
		}
	}

	orb_check(_vehicle_land_detected_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_land_detected), _vehicle_land_detected_sub, &_vehicle_land_detected);
	}

	orb_check(_control_mode_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_control_mode), _control_mode_sub, &_control_mode);
	}

	orb_check(_att_sub, &updated);

	if (updated) {
		vehicle_attitude_s att;
		if (orb_copy(ORB_ID(vehicle_attitude), _att_sub, &att) == PX4_OK && PX4_ISFINITE(att.q[0])) {
			_states.yaw = Eulerf(Quatf(att.q)).psi();
		}
	}

	orb_check(_home_pos_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(home_position), _home_pos_sub, &_home_pos);
	}

	orb_check(_traj_wp_avoidance_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_trajectory_waypoint), _traj_wp_avoidance_sub, &_traj_wp_avoidance);
	}

	// Yuneec-specific
	orb_check(_manual_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(manual_control_setpoint), _manual_sub, &_manual);
	}

	// END Yuneec-specific
}

void
MulticopterPositionControl::limit_altitude(vehicle_local_position_setpoint_s &setpoint)
{
	if (_vehicle_land_detected.alt_max < 0.0f || !_home_pos.valid_alt || !_local_pos.v_z_valid) {
		// there is no altitude limitation present or the required information not available
		return;
	}

	float altitude_above_home = -(_states.position(2) - _home_pos.z);

	if (altitude_above_home > _vehicle_land_detected.alt_max) {
		// we are above maximum altitude
		setpoint.z = -_vehicle_land_detected.alt_max +  _home_pos.z;
		setpoint.vz = 0.0f;

	} else if (setpoint.vz <= 0.0f) {
		// we want to fly upwards: check if vehicle does not exceed altitude

		float delta_p = _vehicle_land_detected.alt_max - altitude_above_home;

		if (fabsf(setpoint.vz) * _dt > delta_p) {
			setpoint.z = -_vehicle_land_detected.alt_max +  _home_pos.z;
			setpoint.vz = 0.0f;
		}
	}
}

void
MulticopterPositionControl::set_vehicle_states(const float &vel_sp_z)
{
	if (_local_pos.timestamp == 0) {
		return;
	}

	// only set position states if valid and finite
	if (PX4_ISFINITE(_local_pos.x) && PX4_ISFINITE(_local_pos.y) && _local_pos.xy_valid) {
		_states.position(0) = _local_pos.x;
		_states.position(1) = _local_pos.y;

	} else {
		_states.position(0) = _states.position(1) = NAN;
	}

	if (PX4_ISFINITE(_local_pos.z) && _local_pos.z_valid) {
		_states.position(2) = _local_pos.z;

	} else {
		_states.position(2) = NAN;
	}

	if (PX4_ISFINITE(_local_pos.vx) && PX4_ISFINITE(_local_pos.vy) && _local_pos.v_xy_valid) {
		_states.velocity(0) = _local_pos.vx;
		_states.velocity(1) = _local_pos.vy;
		_states.acceleration(0) = _vel_x_deriv.update(-_states.velocity(0));
		_states.acceleration(1) = _vel_y_deriv.update(-_states.velocity(1));

	} else {
		_states.velocity(0) = _states.velocity(1) = NAN;
		_states.acceleration(0) = _states.acceleration(1) = NAN;

		// since no valid velocity, update derivate with 0
		_vel_x_deriv.update(0.0f);
		_vel_y_deriv.update(0.0f);
	}

	if (_param_mpc_alt_mode.get() && _local_pos.dist_bottom_valid && PX4_ISFINITE(_local_pos.dist_bottom_rate)) {
		// terrain following
		_states.velocity(2) = -_local_pos.dist_bottom_rate;
		_states.acceleration(2) = _vel_z_deriv.update(-_states.velocity(2));

	} else if (PX4_ISFINITE(_local_pos.vz)) {

		_states.velocity(2) = _local_pos.vz;

		if (PX4_ISFINITE(vel_sp_z) && fabsf(vel_sp_z) > FLT_EPSILON && PX4_ISFINITE(_local_pos.z_deriv)) {
			// A change in velocity is demanded. Set velocity to the derivative of position
			// because it has less bias but blend it in across the landing speed range
			float weighting = fminf(fabsf(vel_sp_z) / _param_mpc_land_speed.get(), 1.0f);
			_states.velocity(2) = _local_pos.z_deriv * weighting + _local_pos.vz * (1.0f - weighting);
		}

		_states.acceleration(2) = _vel_z_deriv.update(-_states.velocity(2));

	} else {
		_states.velocity(2) = _states.acceleration(2) = NAN;
		// since no valid velocity, update derivate with 0
		_vel_z_deriv.update(0.0f);

	}
}

int
MulticopterPositionControl::print_status()
{
	if (_flight_tasks.isAnyTaskActive()) {
		PX4_INFO("Running, active flight task: %i", _flight_tasks.getActiveTask());

	} else {
		PX4_INFO("Running, no flight task active");
	}

	return 0;
}

void
MulticopterPositionControl::run()
{
	hrt_abstime time_stamp_last_loop = hrt_absolute_time(); // time stamp of last loop iteration

	// initialize all subscriptions
	_vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	_vehicle_land_detected_sub = orb_subscribe(ORB_ID(vehicle_land_detected));
	_control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	_att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	_home_pos_sub = orb_subscribe(ORB_ID(home_position));
	_traj_wp_avoidance_sub = orb_subscribe(ORB_ID(vehicle_trajectory_waypoint));
	_manual_sub = orb_subscribe(ORB_ID(manual_control_setpoint));

	// get initial values for all parameters and subscribtions
	parameters_update(true);
	poll_subscriptions();

	// setup file descriptor to poll the local position as loop wakeup source
	px4_pollfd_struct_t poll_fd = {};
	poll_fd.fd = _local_pos_sub;
	poll_fd.events = POLLIN;

	while (!should_exit()) {
		// poll for new data on the local position state topic (wait for up to 20ms)
		const int poll_return = px4_poll(&poll_fd, 1, 20);

		// poll_return == 0: go through the loop anyway to copy manual input at 50 Hz
		// this is undesirable but not much we can do
		if (poll_return < 0) {
			PX4_ERR("poll error %d %d", poll_return, errno);
			continue;
		}

		poll_subscriptions();
		parameters_update(false);

		// set _dt in controllib Block - the time difference since the last loop iteration in seconds
		const hrt_abstime time_stamp_current = hrt_absolute_time();
		setDt((time_stamp_current - time_stamp_last_loop) / 1e6f);
		time_stamp_last_loop = time_stamp_current;

		const bool was_in_failsafe = _in_failsafe;
		_in_failsafe = false;

		// activate the weathervane controller if required. If activated a flighttask can use it to implement a yaw-rate control strategy
		// that turns the nose of the vehicle into the wind
		if (_wv_controller != nullptr) {

			// in manual mode we just want to use weathervane if position is controlled as well
			if (_wv_controller->weathervane_enabled() && (!_control_mode.flag_control_manual_enabled
					|| _control_mode.flag_control_position_enabled)) {
				_wv_controller->activate();

			} else {
				_wv_controller->deactivate();
			}

			_wv_controller->update(matrix::Quatf(_att_sp.q_d), _states.yaw);
		}

		// an update is necessary here because otherwise the takeoff state doesn't get skiped with non-altitude-controlled modes
		_takeoff.updateTakeoffState(_control_mode.flag_armed, _vehicle_land_detected.landed, false, 10.f,
					    !_control_mode.flag_control_climb_rate_enabled, time_stamp_current);

		// takeoff delay for motors to reach idle speed
		if (_takeoff.getTakeoffState() >= TakeoffState::ready_for_takeoff) {
			// when vehicle is ready switch to the required flighttask
			start_flight_task();

		} else {
			// stop flighttask while disarmed
			_flight_tasks.switchTask(FlightTaskIndex::None);
		}

		// check if any task is active
		if (_flight_tasks.isAnyTaskActive()) {

			// setpoints from flighttask
			vehicle_local_position_setpoint_s setpoint;

			_flight_tasks.setYawHandler(_wv_controller);

			// update task
			if (!_flight_tasks.update()) {
				// FAILSAFE
				// Task was not able to update correctly. Do Failsafe.
				failsafe(setpoint, _states, false, !was_in_failsafe);

			} else {
				setpoint = _flight_tasks.getPositionSetpoint();
				_failsafe_land_hysteresis.set_state_and_update(false, time_stamp_current);

				// for logging only: publish flighttask setpoints
				setpoint.timestamp = hrt_absolute_time();
				orb_publish_auto(ORB_ID(flighttask_setpoint), &_flighttask_sp_pub, &setpoint, nullptr, ORB_PRIO_DEFAULT);

				// Check if position, velocity or thrust pairs are valid -> trigger failsaife if no pair is valid
				if (!(PX4_ISFINITE(setpoint.x) && PX4_ISFINITE(setpoint.y)) &&
				    !(PX4_ISFINITE(setpoint.vx) && PX4_ISFINITE(setpoint.vy)) &&
				    !(PX4_ISFINITE(setpoint.thrust[0]) && PX4_ISFINITE(setpoint.thrust[1]))) {
					failsafe(setpoint, _states, true, !was_in_failsafe);
				}

				// Check if altitude, climbrate or thrust in D-direction are valid -> trigger failsafe if none
				// of these setpoints are valid
				if (!PX4_ISFINITE(setpoint.z) && !PX4_ISFINITE(setpoint.vz) && !PX4_ISFINITE(setpoint.thrust[2])) {
					failsafe(setpoint, _states, true, !was_in_failsafe);
				}
			}

			/* desired waypoints for obstacle avoidance:
			 * point_0 contains the current position with the desired velocity
			 * point_1 contains _pos_sp_triplet.current if valid
			 */
			update_avoidance_waypoint_desired(_states, setpoint);

			vehicle_constraints_s constraints = _flight_tasks.getConstraints();
			landing_gear_s gear = _flight_tasks.getGear();

			// check if all local states are valid and map accordingly
			set_vehicle_states(setpoint.vz);

			// handle smooth takeoff
			_takeoff.updateTakeoffState(_control_mode.flag_armed, _vehicle_land_detected.landed, constraints.want_takeoff,
						    constraints.speed_up, !_control_mode.flag_control_climb_rate_enabled, time_stamp_current);
			constraints.speed_up = _takeoff.updateRamp(_dt, constraints.speed_up);

			if (_takeoff.getTakeoffState() < TakeoffState::rampup && !PX4_ISFINITE(setpoint.thrust[2])) {
				// we are not flying yet and need to avoid any corrections
				reset_setpoint_to_nan(setpoint);
				setpoint.thrust[0] = setpoint.thrust[1] = setpoint.thrust[2] = 0.0f;
				// set yaw-sp to current yaw
				// TODO: we need a clean way to disable yaw control
				setpoint.yaw = _states.yaw;
				setpoint.yawspeed = 0.f;
				// prevent any integrator windup
				_control.resetIntegralXY();
				_control.resetIntegralZ();
				// reactivate the task which will reset the setpoint to current state
				_flight_tasks.reActivate();
			}

			if (_takeoff.getTakeoffState() < TakeoffState::flight && !PX4_ISFINITE(setpoint.thrust[2])) {
				constraints.tilt = math::radians(_param_mpc_tiltmax_lnd.get());
			}


			// limit altitude only if local position is valid
			if (PX4_ISFINITE(_states.position(2))) {
				limit_altitude(setpoint);
			}

			// adjust setpoints based on avoidance
			if (use_obstacle_avoidance()) {
				execute_avoidance_waypoint(setpoint);
			}

			// Update states, setpoints and constraints.
			_control.updateConstraints(constraints);
			_control.updateState(_states);

			// for logging only: publish constraints
			constraints.timestamp = hrt_absolute_time();
			orb_publish_auto(ORB_ID(vehicle_constraints), &_constraints_pub, &constraints, nullptr, ORB_PRIO_DEFAULT);

			// update position controller setpoints
			if (!_control.updateSetpoint(setpoint)) {
				warn_rate_limited("Position-Control Setpoint-Update failed");
			}

			// for logging only: publish position-controller  setpoint and state inputs
			setpoint.timestamp = hrt_absolute_time();
			orb_publish_auto(ORB_ID(position_control_setpoint_input), &_position_control_setpoint_input_pub, &setpoint,
					 nullptr, ORB_PRIO_DEFAULT);

			// Generate desired thrust and yaw.
			_control.generateThrustYawSetpoint(_dt);

			publish_trajectory_sp(setpoint);

			// Fill local position, velocity and thrust setpoint.
			// This message contains setpoints where each type of setpoint is either the input to the PositionController
			// or was generated by the PositionController and therefore corresponds to the PositioControl internal states (states that were generated by P-PID).
			// Example:
			// If the desired setpoint is position-setpoint, _local_pos_sp will contain
			// position-, velocity- and thrust-setpoint where the velocity- and thrust-setpoint were generated by the PositionControlller.
			// If the desired setpoint has a velocity-setpoint only, then _local_pos_sp will contain valid velocity- and thrust-setpoint, but the position-setpoint
			// will remain NAN. Given that the PositionController cannot generate a position-setpoint, this type of setpoint is always equal to the input to the
			// PositionController.
			vehicle_local_position_setpoint_s local_pos_sp{};
			local_pos_sp.timestamp = hrt_absolute_time();
			local_pos_sp.x = setpoint.x;
			local_pos_sp.y = setpoint.y;
			local_pos_sp.z = setpoint.z;
			local_pos_sp.yaw = _control.getYawSetpoint();
			local_pos_sp.yawspeed = _control.getYawspeedSetpoint();
			local_pos_sp.vx = PX4_ISFINITE(_control.getVelSp()(0)) ? _control.getVelSp()(0) : setpoint.vx;
			local_pos_sp.vy = PX4_ISFINITE(_control.getVelSp()(1)) ? _control.getVelSp()(1) : setpoint.vy;
			local_pos_sp.vz = PX4_ISFINITE(_control.getVelSp()(2)) ? _control.getVelSp()(2) : setpoint.vz;
			_control.getThrustSetpoint().copyTo(local_pos_sp.thrust);

			// Publish local position setpoint
			// This message will be used by other modules (such as Landdetector) to determine
			// vehicle intention.
			publish_local_pos_sp(local_pos_sp);

			// Part of landing logic: if ground-contact/maybe landed was detected, turn off
			// controller. This message does not have to be logged as part of the vehicle_local_position_setpoint topic.
			// Note: only adust thrust output if there was not thrust-setpoint demand in D-direction.
			if (_takeoff.getTakeoffState() > TakeoffState::rampup && !PX4_ISFINITE(setpoint.thrust[2])) {
				limit_thrust_during_landing(local_pos_sp);
			}

			// Fill attitude setpoint. Attitude is computed from yaw and thrust setpoint.
			_att_sp = ControlMath::thrustToAttitude(matrix::Vector3f(local_pos_sp.thrust), local_pos_sp.yaw);
			_att_sp.yaw_sp_move_rate = _control.getYawspeedSetpoint();
			_att_sp.fw_control_yaw = false;
			_att_sp.apply_flaps = false;

			// publish attitude setpoint
			// Note: this requires review. The reason for not sending
			// an attitude setpoint is because for non-flighttask modes
			// the attitude septoint should come from another source, otherwise
			// they might conflict with each other such as in offboard attitude control.
			publish_attitude();

			// if there's any change in landing gear setpoint publish it
			if (gear.landing_gear != _old_landing_gear_position
			    && gear.landing_gear != landing_gear_s::GEAR_KEEP) {

				_landing_gear.landing_gear = gear.landing_gear;
				_landing_gear.timestamp = hrt_absolute_time();

				if (_landing_gear_pub != nullptr) {
					orb_publish(ORB_ID(landing_gear), _landing_gear_pub, &_landing_gear);

				} else {
					_landing_gear_pub = orb_advertise(ORB_ID(landing_gear), &_landing_gear);
				}
			}

			_old_landing_gear_position = gear.landing_gear;

		} else {
			// no flighttask is active: set attitude setpoint to idle
			_att_sp.roll_body = _att_sp.pitch_body = 0.0f;
			_att_sp.yaw_body = _states.yaw;
			_att_sp.yaw_sp_move_rate = 0.0f;
			_att_sp.fw_control_yaw = false;
			_att_sp.apply_flaps = false;
			matrix::Quatf q_sp = matrix::Eulerf(_att_sp.roll_body, _att_sp.pitch_body, _att_sp.yaw_body);
			q_sp.copyTo(_att_sp.q_d);
			_att_sp.q_d_valid = true;
			_att_sp.thrust_body[2] = 0.0f;
		}
	}

	orb_unsubscribe(_vehicle_status_sub);
	orb_unsubscribe(_vehicle_land_detected_sub);
	orb_unsubscribe(_control_mode_sub);
	orb_unsubscribe(_params_sub);
	orb_unsubscribe(_local_pos_sub);
	orb_unsubscribe(_att_sub);
	orb_unsubscribe(_home_pos_sub);
	orb_unsubscribe(_traj_wp_avoidance_sub);
}

void
MulticopterPositionControl::start_flight_task()
{
	bool task_failure = false;
	bool should_disable_task = true;
	int prev_failure_count = _task_failure_count;

	// Do not run any flight task for VTOLs in fixed-wing mode
	if (!_vehicle_status.is_rotary_wing) {
		_flight_tasks.switchTask(FlightTaskIndex::None);
		return;
	}

	if (_vehicle_status.in_transition_mode) {
		should_disable_task = false;
		int error = _flight_tasks.switchTask(FlightTaskIndex::Transition);

		if (error != 0) {
			if (prev_failure_count == 0) {
				PX4_WARN("Transition activation failed with error: %s", _flight_tasks.errorToString(error));
			}

			task_failure = true;
			_task_failure_count++;

		} else {
			// we want to be in this mode, reset the failure count
			_task_failure_count = 0;
		}

		return;
	}

	// offboard
	if (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD
	    && (_control_mode.flag_control_altitude_enabled ||
		_control_mode.flag_control_position_enabled ||
		_control_mode.flag_control_climb_rate_enabled ||
		_control_mode.flag_control_velocity_enabled ||
		_control_mode.flag_control_acceleration_enabled)) {

		should_disable_task = false;
		int error = _flight_tasks.switchTask(FlightTaskIndex::Offboard);

		if (error != 0) {
			if (prev_failure_count == 0) {
				PX4_WARN("Offboard activation failed with error: %s", _flight_tasks.errorToString(error));
			}

			task_failure = true;
			_task_failure_count++;

		} else {
			// we want to be in this mode, reset the failure count
			_task_failure_count = 0;
		}
	}

	// Auto-follow me
	if (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_FOLLOW_TARGET) {
		should_disable_task = false;
		int error = _flight_tasks.switchTask(FlightTaskIndex::AutoFollowMe);

		if (error != 0) {
			if (prev_failure_count == 0) {
				PX4_WARN("Follow-Me activation failed with error: %s", _flight_tasks.errorToString(error));
			}

			task_failure = true;
			_task_failure_count++;

		} else {
			// we want to be in this mode, reset the failure count
			_task_failure_count = 0;
		}

	} else if (_control_mode.flag_control_auto_enabled) {
		// Auto related maneuvers
		should_disable_task = false;
		int error = 0;

		switch (_param_mpc_auto_mode.get()) {
		case 1:
			error =  _flight_tasks.switchTask(FlightTaskIndex::AutoLineSmoothVel);
			break;

		default:
			error =  _flight_tasks.switchTask(FlightTaskIndex::AutoLine);
			break;
		}

		if (error != 0) {
			if (prev_failure_count == 0) {
				PX4_WARN("Auto activation failed with error: %s", _flight_tasks.errorToString(error));
			}

			task_failure = true;
			_task_failure_count++;

		} else {
			// we want to be in this mode, reset the failure count
			_task_failure_count = 0;
		}
	}

	// manual position control
	if (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_POSCTL || task_failure) {
		should_disable_task = false;
		int error = 0;

		switch (_param_mpc_pos_mode.get()) {
		case 1:
			error =  _flight_tasks.switchTask(FlightTaskIndex::ManualPositionSmooth);
			break;

		case 2:
			error =  _flight_tasks.switchTask(FlightTaskIndex::Sport);
			break;

		case 3:
			error =  _flight_tasks.switchTask(FlightTaskIndex::ManualPositionSmoothVel);
			break;

		default:
			error =  _flight_tasks.switchTask(FlightTaskIndex::ManualPosition);
			break;
		}

		if (error != 0) {
			if (prev_failure_count == 0) {
				PX4_WARN("Position-Ctrl activation failed with error: %s", _flight_tasks.errorToString(error));
			}

			task_failure = true;
			_task_failure_count++;

		} else {
			check_failure(task_failure, vehicle_status_s::NAVIGATION_STATE_POSCTL);
			task_failure = false;
		}
	}

	// manual altitude control
	if (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_ALTCTL || task_failure) {
		should_disable_task = false;
		int error = 0;

		switch (_param_mpc_pos_mode.get()) {
		case 1:
			error =  _flight_tasks.switchTask(FlightTaskIndex::ManualAltitudeSmooth);
			break;

		case 3:
			error =  _flight_tasks.switchTask(FlightTaskIndex::ManualAltitudeSmoothVel);
			break;

		default:
			error =  _flight_tasks.switchTask(FlightTaskIndex::ManualAltitude);
			break;
		}

		if (error != 0) {
			if (prev_failure_count == 0) {
				PX4_WARN("Altitude-Ctrl activation failed with error: %s", _flight_tasks.errorToString(error));
			}

			task_failure = true;
			_task_failure_count++;

		} else {
			check_failure(task_failure, vehicle_status_s::NAVIGATION_STATE_ALTCTL);
			task_failure = false;
		}
	}

	if (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_ORBIT) {
		should_disable_task = false;
	}

	// check task failure
	if (task_failure) {

		// for some reason no flighttask was able to start.
		// go into failsafe flighttask
		int error = _flight_tasks.switchTask(FlightTaskIndex::Failsafe);

		if (error != 0) {
			// No task was activated.
			_flight_tasks.switchTask(FlightTaskIndex::None);
		}

	} else if (should_disable_task) {
		_flight_tasks.switchTask(FlightTaskIndex::None);
	}
}

void
MulticopterPositionControl::limit_thrust_during_landing(vehicle_local_position_setpoint_s &setpoint)
{
	if (_vehicle_land_detected.ground_contact
	    || _vehicle_land_detected.maybe_landed) {
		// we set thrust to zero, this will help to decide if we are actually landed or not
		setpoint.thrust[0] = setpoint.thrust[1] = setpoint.thrust[2] = 0.0f;
		// set yaw-sp to current yaw to avoid any corrections
		setpoint.yaw = _states.yaw;
		setpoint.yawspeed = 0.f;
		// prevent any integrator windup
		_control.resetIntegralXY();
		_control.resetIntegralZ();
	}
}

void
MulticopterPositionControl::failsafe(vehicle_local_position_setpoint_s &setpoint, const PositionControlStates &states,
				     const bool force, const bool warn)
{
	_failsafe_land_hysteresis.set_state_and_update(true, hrt_absolute_time());

	if (!_failsafe_land_hysteresis.get_state() && !force) {
		// just keep current setpoint and don't do anything.

	} else {
		reset_setpoint_to_nan(setpoint);

		if (PX4_ISFINITE(_states.velocity(2))) {
			// We have a valid velocity in D-direction.
			// descend downwards with landspeed.
			setpoint.vz = _param_mpc_land_speed.get();
			setpoint.thrust[0] = setpoint.thrust[1] = 0.0f;

			if (warn) {
				PX4_WARN("Failsafe: Descend with land-speed.");
			}

		} else {
			// Use the failsafe from the PositionController.
			if (warn) {
				PX4_WARN("Failsafe: Descend with just attitude control.");
			}
		}

		_in_failsafe = true;
	}
}

void
MulticopterPositionControl::update_avoidance_waypoint_desired(PositionControlStates &states,
		vehicle_local_position_setpoint_s &setpoint)
{
	if ((_manual.obsavoid_switch == manual_control_setpoint_s::SWITCH_POS_ON)
	    || (_manual.obsavoid_switch == manual_control_setpoint_s::SWITCH_POS_MIDDLE)) {

		_traj_wp_avoidance_desired = _flight_tasks.getAvoidanceWaypoint();

		_traj_wp_avoidance_desired.timestamp = hrt_absolute_time();
		_traj_wp_avoidance_desired.type = vehicle_trajectory_waypoint_s::MAV_TRAJECTORY_REPRESENTATION_WAYPOINTS;

		_traj_wp_avoidance_desired.waypoints[vehicle_trajectory_waypoint_s::POINT_0].position[0] = setpoint.x;
		_traj_wp_avoidance_desired.waypoints[vehicle_trajectory_waypoint_s::POINT_0].position[1] = setpoint.y;
		_traj_wp_avoidance_desired.waypoints[vehicle_trajectory_waypoint_s::POINT_0].position[2] = setpoint.z;
		_traj_wp_avoidance_desired.waypoints[vehicle_trajectory_waypoint_s::POINT_0].velocity[0] = setpoint.vx;
		_traj_wp_avoidance_desired.waypoints[vehicle_trajectory_waypoint_s::POINT_0].velocity[1] = setpoint.vy;
		_traj_wp_avoidance_desired.waypoints[vehicle_trajectory_waypoint_s::POINT_0].velocity[2] = setpoint.vz;
		states.acceleration.copyTo(_traj_wp_avoidance_desired.waypoints[vehicle_trajectory_waypoint_s::POINT_0].acceleration);
		_traj_wp_avoidance_desired.waypoints[vehicle_trajectory_waypoint_s::POINT_0].yaw = states.yaw;
		_traj_wp_avoidance_desired.waypoints[vehicle_trajectory_waypoint_s::POINT_0].yaw_speed = NAN;
		_traj_wp_avoidance_desired.waypoints[vehicle_trajectory_waypoint_s::POINT_0].point_valid = true;

		publish_avoidance_desired_waypoint();
	}
}

void
MulticopterPositionControl::execute_avoidance_waypoint(vehicle_local_position_setpoint_s &setpoint)
{
	setpoint.x = _traj_wp_avoidance.waypoints[vehicle_trajectory_waypoint_s::POINT_0].position[0];
	setpoint.y = _traj_wp_avoidance.waypoints[vehicle_trajectory_waypoint_s::POINT_0].position[1];
	setpoint.z = _traj_wp_avoidance.waypoints[vehicle_trajectory_waypoint_s::POINT_0].position[2];

	setpoint.vx = _traj_wp_avoidance.waypoints[vehicle_trajectory_waypoint_s::POINT_0].velocity[0];
	setpoint.vy = _traj_wp_avoidance.waypoints[vehicle_trajectory_waypoint_s::POINT_0].velocity[1];
	setpoint.vz = _traj_wp_avoidance.waypoints[vehicle_trajectory_waypoint_s::POINT_0].velocity[2];

	setpoint.acc_x = setpoint.acc_y = setpoint.acc_z = NAN;

	setpoint.yaw = _traj_wp_avoidance.waypoints[vehicle_trajectory_waypoint_s::POINT_0].yaw;
	setpoint.yawspeed = _traj_wp_avoidance.waypoints[vehicle_trajectory_waypoint_s::POINT_0].yaw_speed;
	Vector3f(NAN, NAN, NAN).copyTo(setpoint.thrust);
}

void
MulticopterPositionControl::reset_setpoint_to_nan(vehicle_local_position_setpoint_s &setpoint)
{
	setpoint.x = setpoint.y = setpoint.z = NAN;
	setpoint.vx = setpoint.vy = setpoint.vz = NAN;
	setpoint.yaw = setpoint.yawspeed = NAN;
	setpoint.acc_x = setpoint.acc_y = setpoint.acc_z = NAN;
	setpoint.thrust[0] = setpoint.thrust[1] = setpoint.thrust[2] = NAN;
}

bool
MulticopterPositionControl::use_obstacle_avoidance()
{
	if (_manual.obsavoid_switch == manual_control_setpoint_s::SWITCH_POS_ON
		 || _manual.obsavoid_switch == manual_control_setpoint_s::SWITCH_POS_MIDDLE) {
		const bool avoidance_data_timeout = hrt_elapsed_time((hrt_abstime *)&_traj_wp_avoidance.timestamp) > TRAJECTORY_STREAM_TIMEOUT_US;
		const bool avoidance_point_valid = _traj_wp_avoidance.waypoints[vehicle_trajectory_waypoint_s::POINT_0].point_valid == true;
		const bool in_mission = _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION;
		const bool in_rtl = _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_RTL;
		const bool in_pos_ctl = _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_POSCTL;

		// switch to hold mode to stop when we loose external avoidance data during a mission
		if (avoidance_data_timeout && in_mission) {
			send_vehicle_cmd_do(vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER);
		}

		if ((in_mission || in_rtl || in_pos_ctl) && !avoidance_data_timeout && avoidance_point_valid) {
			return true;
		}
	}
	return false;
}

void
MulticopterPositionControl::publish_avoidance_desired_waypoint()
{
	// publish desired waypoint
	if (_traj_wp_avoidance_desired_pub != nullptr) {
		orb_publish(ORB_ID(vehicle_trajectory_waypoint_desired), _traj_wp_avoidance_desired_pub, &_traj_wp_avoidance_desired);

	} else {
		_traj_wp_avoidance_desired_pub = orb_advertise(ORB_ID(vehicle_trajectory_waypoint_desired),
						 &_traj_wp_avoidance_desired);
	}
}

void
MulticopterPositionControl::publish_attitude()
{
	_att_sp.timestamp = hrt_absolute_time();

	if (_att_sp_pub != nullptr) {
		orb_publish(_attitude_setpoint_id, _att_sp_pub, &_att_sp);

	} else if (_attitude_setpoint_id) {
		_att_sp_pub = orb_advertise(_attitude_setpoint_id, &_att_sp);
	}
}

void
MulticopterPositionControl::publish_trajectory_sp(const vehicle_local_position_setpoint_s &traj)
{
	// publish trajectory
	if (_traj_sp_pub != nullptr) {
		orb_publish(ORB_ID(trajectory_setpoint), _traj_sp_pub, &traj);

	} else {
		_traj_sp_pub = orb_advertise(ORB_ID(trajectory_setpoint), &traj);
	}
}

void
MulticopterPositionControl::publish_local_pos_sp(const vehicle_local_position_setpoint_s &local_pos_sp)
{
	// publish local position setpoint
	if (_local_pos_sp_pub != nullptr) {
		orb_publish(ORB_ID(vehicle_local_position_setpoint), _local_pos_sp_pub, &local_pos_sp);

	} else {
		_local_pos_sp_pub = orb_advertise(ORB_ID(vehicle_local_position_setpoint), &local_pos_sp);
	}
}

void MulticopterPositionControl::check_failure(bool task_failure, uint8_t nav_state)
{
	if (!task_failure) {
		// we want to be in this mode, reset the failure count
		_task_failure_count = 0;

	} else if (_task_failure_count > NUM_FAILURE_TRIES) {
		// tell commander to switch mode
		PX4_WARN("Previous flight task failed, switching to mode %d", nav_state);
		send_vehicle_cmd_do(nav_state);
		_task_failure_count = 0; // avoid immediate resending of a vehicle command in the next iteration
	}
}

void MulticopterPositionControl::send_vehicle_cmd_do(uint8_t nav_state)
{
	vehicle_command_s command{};
	command.timestamp = hrt_absolute_time();
	command.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
	command.param1 = (float)1; // base mode
	command.param3 = (float)0; // sub mode
	command.target_system = 1;
	command.target_component = 1;
	command.source_system = 1;
	command.source_component = 1;
	command.confirmation = false;
	command.from_external = false;

	// set the main mode
	switch (nav_state) {
	case vehicle_status_s::NAVIGATION_STATE_STAB:
		command.param2 = (float)PX4_CUSTOM_MAIN_MODE_STABILIZED;
		break;

	case vehicle_status_s::NAVIGATION_STATE_ALTCTL:
		command.param2 = (float)PX4_CUSTOM_MAIN_MODE_ALTCTL;
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER:
		command.param2 = (float)PX4_CUSTOM_MAIN_MODE_AUTO;
		command.param3 = (float)PX4_CUSTOM_SUB_MODE_AUTO_LOITER;
		break;

	default: //vehicle_status_s::NAVIGATION_STATE_POSCTL
		command.param2 = (float)PX4_CUSTOM_MAIN_MODE_POSCTL;
		break;
	}

	// publish the vehicle command
	if (_pub_vehicle_command == nullptr) {
		_pub_vehicle_command = orb_advertise_queue(ORB_ID(vehicle_command), &command,
				       vehicle_command_s::ORB_QUEUE_LENGTH);

	} else {
		orb_publish(ORB_ID(vehicle_command), _pub_vehicle_command, &command);
	}
}

int MulticopterPositionControl::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("mc_pos_control",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_POSITION_CONTROL,
				      1900,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

MulticopterPositionControl *MulticopterPositionControl::instantiate(int argc, char *argv[])
{
	return new MulticopterPositionControl();
}

int MulticopterPositionControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int mc_pos_control_main(int argc, char *argv[])
{
	return MulticopterPositionControl::main(argc, argv);
}
