/****************************************************************************
 *
 *   Copyright (c) 2012-2018 PX4 Development Team. All rights reserved.
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
 * @file sensors.h
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Julian Oes <julian@oes.ch>
 * @author Thomas Gubler <thomas@px4.io>
 * @author Anton Babushkin <anton@px4.io>
 * @author Beat Küng <beat-kueng@gmx.net>
 */

#include <board_config.h>

#include <px4_config.h>
#include <px4_module.h>
#include <px4_module_params.h>
#include <px4_getopt.h>
#include <px4_posix.h>
#include <px4_tasks.h>
#include <px4_time.h>

#include <fcntl.h>
#include <poll.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <errno.h>
#include <math.h>
#include <mathlib/mathlib.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_rc_input.h>
#include <drivers/drv_adc.h>
#include <drivers/drv_airspeed.h>

#include <airspeed/airspeed.h>
#include <parameters/param.h>
#include <systemlib/err.h>
#include <perf/perf_counter.h>
#include <battery/battery.h>

#include <conversion/rotation.h>

#include <uORB/uORB.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/differential_pressure.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/sensor_preflight.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/vehicle_magnetometer.h>

#include <DevMgr.hpp>

#include "parameters.h"
#include "rc_update.h"
#include "voted_sensors_update.h"

#pragma once

using namespace DriverFramework;
using namespace time_literals;

class Sensors : public ModuleBase<Sensors>, public ModuleParams
{
public:
	Sensors(bool hil_enabled);
	~Sensors() = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static Sensors *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

private:
	DevHandle 	_h_adc;				/**< ADC driver handle */

	hrt_abstime	_last_adc{0};			/**< last time we took input from the ADC */

	const bool	_hil_enabled;			/**< if true, HIL is active */
	bool		_armed{false};				/**< arming status of the vehicle */

	int		_actuator_ctrl_0_sub{-1};		/**< attitude controls sub */
	int		_diff_pres_sub{-1};			/**< raw differential pressure subscription */
	int		_vcontrol_mode_sub{-1};		/**< vehicle control mode subscription */
	int 		_params_sub{-1};			/**< notification of parameter updates */

	orb_advert_t	_sensor_pub{nullptr};			/**< combined sensor data topic */
	orb_advert_t	_airdata_pub{nullptr};			/**< combined sensor data topic */
	orb_advert_t	_magnetometer_pub{nullptr};			/**< combined sensor data topic */

#if BOARD_NUMBER_BRICKS > 0
	orb_advert_t	_battery_pub[BOARD_NUMBER_BRICKS] {};			/**< battery status */

	Battery		_battery[BOARD_NUMBER_BRICKS];			/**< Helper lib to publish battery_status topic. */
#endif /* BOARD_NUMBER_BRICKS > 0 */

#if BOARD_NUMBER_BRICKS > 1
	int 			_battery_pub_intance0ndx {0}; /**< track the index of instance 0 */
#endif /* BOARD_NUMBER_BRICKS > 1 */

	orb_advert_t	_airspeed_pub{nullptr};			/**< airspeed */
	orb_advert_t	_sensor_preflight{nullptr};		/**< sensor preflight topic */

	perf_counter_t	_loop_perf;			/**< loop performance counter */

	DataValidator	_airspeed_validator;		/**< data validator to monitor airspeed */

#ifdef ADC_AIRSPEED_VOLTAGE_CHANNEL
	differential_pressure_s	_diff_pres {};

	orb_advert_t	_diff_pres_pub{nullptr};			/**< differential_pressure */
#endif /* ADC_AIRSPEED_VOLTAGE_CHANNEL */

	Parameters		_parameters{};			/**< local copies of interesting parameters */
	ParameterHandles	_parameter_handles{};		/**< handles for interesting parameters */

	RCUpdate		_rc_update;
	VotedSensorsUpdate _voted_sensors_update;


	/**
	 * Update our local parameter cache.
	 */
	int		parameters_update();

	/**
	 * Do adc-related initialisation.
	 */
	int		adc_init();

	/**
	 * Poll the differential pressure sensor for updated data.
	 *
	 * @param raw			Combined sensor data structure into which
	 *				data should be returned.
	 */
	void		diff_pres_poll(const vehicle_air_data_s &airdata);

	/**
	 * Check for changes in vehicle control mode.
	 */
	void		vehicle_control_mode_poll();

	/**
	 * Check for changes in parameters.
	 */
	void 		parameter_update_poll(bool forced = false);

	/**
	 * Poll the ADC and update readings to suit.
	 *
	 * @param raw			Combined sensor data structure into which
	 *				data should be returned.
	 */
	void		adc_poll();
};
