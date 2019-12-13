/****************************************************************************
 *
 *   Copyright (c) 2016-2019, 2019 PX4 Development Team. All rights reserved.
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
 * @file battery_base.h
 *
 * Base class for battery estimation.
 *
 */

#pragma once

#include <uORB/topics/battery_status.h>
#include <drivers/drv_hrt.h>
#include <px4_module_params.h>
#include <drivers/drv_hrt.h>
#include <px4_defines.h>

class Battery : public ModuleParams
{
public:

	Battery() :
		ModuleParams(nullptr)
	{}

	virtual ~Battery() = default;

	/**
	 * Reset all battery stats and report invalid/nothing.
	 */
	void reset(battery_status_s *battery_status);

	/**
	 * Get the battery cell count
	 */
	int cell_count() { return _n_cells.get(); }

	/**
	 * Get the empty voltage per cell
	 */
	float empty_cell_voltage() { return _v_empty.get(); }

	/**
	 * Get the full voltage per cell
	 */
	float full_cell_voltage() { return _v_charged.get(); }

	/**
	 * Update current battery status message.
	 *
	 * @param voltage_v: current voltage in V
	 * @param current_a: current current in A
	 * @param connected: Battery is connected
	 * @param selected_source: This battery is on the brick that the selected source for selected_source
	 * @param priority: The brick number -1. The term priority refers to the Vn connection on the LTC4417
	 * @param throttle_normalized: throttle from 0 to 1
	 */
	virtual void updateStatus(hrt_abstime timestamp,
			    float voltage_v,
			    float current_a,
			    bool connected,
			    bool selected_source,
			    int priority,
			    float throttle_normalized,
			    bool armed,
			    battery_status_s *status) = 0;

protected:
	void filter1order(float &signal_filtered, const float signal_raw, const float alpha);
	void determineWarning(uint8_t &warning, const bool &connected, const float &remaining);

	DEFINE_PARAMETERS_CUSTOM_PARENT(ModuleParams,
					(ParamFloat<px4::params::BAT_V_EMPTY>) _v_empty,
					(ParamFloat<px4::params::BAT_V_CHARGED>) _v_charged,
					(ParamInt<px4::params::BAT_N_CELLS>) _n_cells,
					(ParamFloat<px4::params::BAT_LOW_THR>) _low_thr,
					(ParamFloat<px4::params::BAT_CRIT_THR>) _crit_thr,
					(ParamFloat<px4::params::BAT_EMERGEN_THR>) _emergency_thr)
};
