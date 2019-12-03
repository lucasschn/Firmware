/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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
 * @file battery.cpp
 *
 * Library calls for battery functionality.
 *
 * @author Julian Oes <julian@oes.ch>
 */

#include "battery.h"

using namespace matrix;

// Yuneec-specific
#include <systemlib/mavlink_log.h>

Battery::Battery() :
	ModuleParams(nullptr),
	_warning(battery_status_s::BATTERY_WARNING_NONE),
	_last_timestamp(0)
{
}

void
Battery::reset(battery_status_s *battery_status)
{
	memset(battery_status, 0, sizeof(*battery_status));
	battery_status->current_a = -1.f;
	battery_status->remaining = 1.f;
	battery_status->scale = 1.f;
	battery_status->cell_count = _n_cells.get();
	// TODO: check if it is sane to reset warning to NONE
	battery_status->warning = battery_status_s::BATTERY_WARNING_NONE;
	battery_status->connected = false;
}

void
Battery::updateBatteryStatus(hrt_abstime timestamp, float voltage_v, float current_a,
			     bool connected, bool selected_source, int priority,
			     float throttle_normalized,
			     bool armed, battery_status_s *battery_status)
{
	if (_use_kf.get()) {

		if (_init_kf) {
			this->kfInit(voltage_v);
			_init_kf = false; // mark the Kalman filter initialization as don
		}

		reset(battery_status);
		battery_status->timestamp = timestamp;

		filterVoltage(voltage_v);
		filterCurrent(current_a);

		if (_voltage_filtered_v / _n_cells.get() < 3.4f) {
			PX4_WARN("Battery is dangerously low. Land immediately to avoid deep discharge.");
		}

		kfUpdate(timestamp, _current_filtered_a, voltage_v);
		// do estimateRemaining anyways, case is determined inside it.
		estimateRemaining(_voltage_filtered_v, _current_filtered_a, _throttle_filtered, armed);

		bool reliably_connected = true;

#ifdef BOARD_HAS_POWER_CHECK

		if (_link_check.get()) {
			reliably_connected = !stm32_gpioread(POWER_CHECK_GPIO);
		}

#endif

	} else {

		reset(battery_status);
		battery_status->timestamp = timestamp;

		filterVoltage(voltage_v);
		filterCurrent(current_a);
		filterThrottle(throttle_normalized);

		if (_voltage_filtered_v / _n_cells.get() < 3.4f) {
			PX4_WARN("Battery is dangerously low. Land immediately to avoid deep discharge.");
		}


		sumDischarged(timestamp, current_a); // computes _discharged_mah

		// PX4_INFO("current : %f", (double) _current_filtered_a);
		// do estimateRemaining anyways, case is determined inside it.
		estimateRemaining(_voltage_filtered_v, _current_filtered_a, _throttle_filtered, armed);
		computeScale(); // can't figure out what that is
		computeRemainingTime(current_a);

		if (_battery_initialized) {
			determineWarning(connected);
		}

		bool reliably_connected = true;

#ifdef BOARD_HAS_POWER_CHECK

		if (_link_check.get()) {
			reliably_connected = !stm32_gpioread(POWER_CHECK_GPIO);
		}

#endif
	}

	if (_voltage_filtered_v > 2.1f) {
		_battery_initialized = true;
		battery_status->voltage_v = voltage_v;
		battery_status->voltage_filtered_v = _voltage_filtered_v;
		battery_status->current_a = current_a;
		battery_status->current_filtered_a = _current_filtered_a;
		battery_status->discharged_mah = _discharged_mah;
		battery_status->warning = _warning;
		battery_status->remaining = _remaining;
		battery_status->connected = connected;
		battery_status->system_source = selected_source;
		battery_status->priority = priority;
		battery_status->reliably_connected = reliably_connected;
		battery_status->tethered = _tethered;

		if (_use_kf) {
			battery_status->covx[0] = _covx(0, 0);
			battery_status->covx[1] = _covx(0, 1);
			battery_status->covx[2] = _covx(1, 0);
			battery_status->covx[3] = _covx(1, 1);
			battery_status->covw[0] = _covw(0, 0);
			battery_status->covw[1] = _covw(0, 1);
			battery_status->covw[2] = _covw(1, 0);
			battery_status->covw[3] = _covw(1, 1);
			battery_status->kalman_gain[0] = _kalman_gain(0, 0);
			battery_status->kalman_gain[1] = _kalman_gain(1, 0);
			battery_status->innovation = _innovation;
			battery_status->unsaturated_innovation = _y - _yhat;
			battery_status->resistor_current = _xhat(1);

		} else {
			battery_status->scale = _scale;
			battery_status->time_remaining_s = _time_remaining_s;
		}
	}

	// Yuneec-specific [ch4207]: Consider voltage above usual battery voltrage
	// as an infinite power supply. Can be used for tethering for example.
	if (_v_tether.get() >= 0.0f && _voltage_filtered_v >= _v_tether.get()) {
		if (!_tethered) {
			_tethered = true;
			mavlink_log_info(&_mavlink_log_pub, "Tethered power supply: enabled");
		}

		// Override battery estimate while thethered
		battery_status->time_remaining_s = -1.f;
		battery_status->warning = battery_status_s::BATTERY_WARNING_NONE;
		battery_status->remaining = 1.f;
	}

	// For now we don't support switching from thethering to a backup battery
	// else if (_tethered) {
	// 	_tethered  = false;
	// 	_discharged_mah = 0;  // reset internal state when tether disconnects
	// 	mavlink_log_info(&_mavlink_log_pub, "Tethered power supply: disabled");
	// }
}

float Battery::ocv_from_soc(float SOC)
{
	float OCV = 0;

	for (int poly_expo = 0; poly_expo <= sizeof(_poly_coeff) - 1; poly_expo++) {
		OCV += _poly_coeff[sizeof(_poly_coeff) - 1 - poly_expo] * powf(SOC, (float) poly_expo);
	}

	return OCV;

}

float Battery::soc_from_ocv(float OCV)
{
	float SOC = -1;

	/* The OCV(SOC) curve is linearized by parts in three segments. Those values are stored in the _OCV_array and _SOC_array variables. The SOC is computed by linear interpolation, once the OCV has been identified to belong to one of these three segments through the if statements.
	*/
	if (OCV >= _OCV_array[0] and OCV <= _OCV_array[3]) {
		if (OCV >= _OCV_array[0] and OCV <= _OCV_array[1]) {
			SOC = (_SOC_array[1] - _SOC_array[0]) / (_OCV_array[1] - _OCV_array[0]) * (OCV - _OCV_array[0]);

		} else if (OCV > _OCV_array[1] and OCV <= _OCV_array[2]) {
			SOC = (_SOC_array[2] - _SOC_array[1]) / (_OCV_array[2] - _OCV_array[1]) * (OCV - _OCV_array[1]) + _SOC_array[1];

		} else if (OCV > _OCV_array[2] and OCV <= _OCV_array[3]) {
			SOC = (_SOC_array[3] - _SOC_array[2]) / (_OCV_array[3] - _OCV_array[2]) * (OCV - _OCV_array[2]) + _SOC_array[2];
		}
	}

	static bool doprint = true;

	if (doprint) {
		PX4_INFO("Initial voltage : %.2f", (double) v);
		PX4_INFO("Initial SOC : %.2f", (double) SOC);

		if (SOC >= -1) {
			doprint = false;
		}
	}

	return SOC;
}

float Battery::getSlope(float z)
{
	using math::min;
	using math::max;
	float const dSOC = min(1.0f, z + 0.01f) - max(0.0f, z - 0.01f);
	// min(1.0,z+0.01) is used to prevent z+0.01 exceeding 1 in case z>0.99. Having SOC>1 does not make any sense. Same on the other end of the curve.
	// +/-0.01 is an arbitrary value that give decent slope predictions in Jupyter notebook implementation.
	float const dOCV = this->ocv_from_soc(min(1.0f, z + 0.01f)) - this->ocv_from_soc(max(0.0f, z - 0.01f));
	float slope = dOCV / dSOC;

	return slope;
}

void Battery::recompute_statespace(float timedelta)
{
	_batmat_A(1, 1) = exp(-timedelta / (_R1.get() * _C1.get()));
	_batmat_B(0) = timedelta / (_capacity_mAh * 3.6f); // 3.6 converts from mAh to Coulombs
	_batmat_B(1) = 1 - exp(-timedelta / (_R1.get() * _C1.get()));
	_batmat_C(0, 0) = this->getSlope(_xhat(0));
}

void Battery::kfInit(float voltage_v)
{
	_yhat = voltage_v / _n_cells.get();
	// Battery is considered at equilibrium when booting up (if last value from last time could be saved would be nice). That means the sensed voltage is assumed to be equal to the Open Circuit Voltage (OCV).
	_z0 = this->soc_from_ocv(voltage_v / _n_cells.get());
	_iR10 = 0.0f;
	// Matrices initialization
	_batmat_A(0, 0) = 1.0f;
	_batmat_A(0, 1) = 0.0f;
	_batmat_A(1, 0) = 0.0f;
	_batmat_A(1, 1) = exp(-_deltatime / (_R1.get() * _C1.get()));
	_batmat_C(0, 0) = this->getSlope(_z0);
	_batmat_C(0, 1) = -_R1.get();
	_xhat(0) = _z0;
	_xhat(1) = _iR10;
	_covx(0, 0) = 0.1f;
	_covx(0, 1) = 0.f;
	_covx(1, 0) = 0.f;
	_covx(1, 1) = 0.f;
	_covw(0, 0) = 1e-3f;
	_covw(0, 1) = 0.0f;
	_covw(1, 0) = 0.0f;
	_covw(1, 1) = 1e-4f;
	PX4_INFO("Kalman filter has been initialized.");
}

void Battery::kfUpdate(hrt_abstime timestamp, float current_a, float voltage_v)
{
	_u = current_a;
	_y = voltage_v / _n_cells.get(); // Voltage per cell is required
	_deltatime = (timestamp - _last_timestamp) / 1e6;
	this->recompute_statespace(_deltatime);
	SquareMatrix<float, 1> tmp;

	// 1c Computation of future yhat using future state and last input would not make sense !
	_yhat = ocv_from_soc(_xhat(0)) - _R1.get() * _xhat(1) - _R0.get() * _u;

	// 1b
	_covx = _batmat_A * _covx * _batmat_A.T() + _covw; // (numerical robustness) no risk of losing symetry here

	// 2a
	covxy = _covx * _batmat_C.T();
	tmp = _batmat_C * _covx * _batmat_C.T();
	_covy = tmp(0, 0) + _covv; // scalar, so not computationally demanding
	_kalman_gain = covxy / _covy;


	_innovation = _y - _yhat;

	// Innovation saturation
	if (_innovation > 0.05f) {
		_innovation = 0.05f;

	} else if (_innovation < -0.05f) {
		_innovation = -0.05f;
	}

	// 1a & 2b Computation of xhat using last input
	_xhat = _batmat_A * _xhat + _batmat_B * _u + _kalman_gain * _innovation;

	/* Is it really the best way to constraint the state ?
	We could also project the unconstrainted state space
	onto a constrained one using a clever projection
	*/
	if (_xhat(0) > 1.0f) {
		_xhat(0) = 1.0f;

	} else if (_xhat(0) < 0.0f) {
		_xhat(0) = 0.0f;
	}

	// 2c (Joseph-form covariance update for improved numerical robustness)
	_covx = (I - _kalman_gain * _batmat_C) * _covx * (I - _kalman_gain * _batmat_C).T() + _kalman_gain * _covv *
		_kalman_gain.T(); // puts nonzero values in the antidiagonal of covx
	_last_timestamp = timestamp;
}

// For now we don't support switching from thethering to a backup battery
// else if (_tethered) {
// 	_tethered  = false;
// 	_discharged_mah = 0;  // reset internal state when tether disconnects
// 	mavlink_log_info(&_mavlink_log_pub, "Tethered power supply: disabled");
// }


void
Battery::filterVoltage(float voltage_v)
{
	if (!_battery_initialized) {
		PX4_INFO("Battery has not been initialized yet.");
		_voltage_filtered_v = voltage_v;

	} else {
	}


	// TODO: inspect that filter performance
	const float filtered_next = _voltage_filtered_v * 0.99f + voltage_v * 0.01f;

	// static size_t kfilterVoltage = 0;
	if (PX4_ISFINITE(filtered_next)) {
		_voltage_filtered_v = filtered_next;
		// if (kfilterVoltage++ % 100 == 0){
		// 	PX4_INFO("Voltage filtered : %f V", (double) _voltage_filtered_v);
		// }
	}
}

void
Battery::filterCurrent(float current_a)
{
	if (!_battery_initialized) {
		_current_filtered_a = current_a;
	}

	// ADC poll is at 100Hz, this will perform a low pass over approx 500ms
	const float filtered_next = _current_filtered_a * 0.98f + current_a * 0.02f;

	// static size_t kfilterCurrent = 0;

	if (PX4_ISFINITE(filtered_next)) {
		_current_filtered_a = filtered_next;

		// if (kfilterCurrent++ % 10 == 0) {
		// 	PX4_INFO("Current filtered : %f A", (double) _current_filtered_a);
		// }
	}
}

void Battery::filterThrottle(float throttle)
{
	if (!_battery_initialized) {
		_throttle_filtered = throttle;
	}

	const float filtered_next = _throttle_filtered * 0.99f + throttle * 0.01f;

	if (PX4_ISFINITE(filtered_next)) {
		_throttle_filtered = filtered_next;
	}
}

// not executed if use_kf=true
void
Battery::sumDischarged(hrt_abstime timestamp, float current_a)
{
	// Not a valid measurement
	if (current_a < 0.f) {
		// Because the measurement was invalid we need to stop integration
		// and re-initialize with the next valid measurement
		_last_timestamp = 0;
		return;
	}

	// Ignore first update because we don't know dt.
	if (_last_timestamp != 0) {
		_deltatime = (timestamp - _last_timestamp) / 1e6;
		// mAh since last loop: (current[A] * 1000 = [mA]) * (dt[s] / 3600 = [h])
		_discharged_mah_loop = (current_a * 1e3f) * (_deltatime / 3600.f);
		_discharged_mah += _discharged_mah_loop;
	}

	_last_timestamp = timestamp;
}

void
Battery::estimateRemaining(float voltage_v, float current_a, float throttle, bool armed)
{
	if (_use_kf.get()) {
		_remaining = _xhat(0);
		// PX4_INFO("SOC : %f", (double)  _remaining);

	} else { // if the Kalman filter is not used
		// remaining battery capacity based on voltage
		float cell_voltage = voltage_v / _n_cells.get();

		// correct battery voltage locally for load drop to avoid estimation fluctuations
		if (_r_internal.get() >= 0.f) {
			cell_voltage += _r_internal.get() * current_a;

		} else {
			// assume linear relation between throttle and voltage drop
			cell_voltage += throttle * _v_load_drop.get();
		}

		_remaining_voltage = math::gradual(cell_voltage, _v_empty.get(), _v_charged.get(), 0.f, 1.f);

		// choose which quantity we're using for final reporting
		if (_capacity.get() > 0.f) {
			// if battery capacity is known, fuse voltage measurement with used capacity
			if (!_battery_initialized) {
				// initialization of the estimation state
				_remaining = _remaining_voltage;

			} else {
				// The lower the voltage the more adjust the estimate with it to avoid deep discharge
				const float weight_v = 3e-4f * (1 - _remaining_voltage);
				_remaining = (1 - weight_v) * _remaining + weight_v * _remaining_voltage;
				// directly apply current capacity slope calculated using current
				_remaining -= _discharged_mah_loop / _capacity.get();
				_remaining = math::max(_remaining, 0.f);
			}

		} else {
			// else use voltage
			_remaining = _remaining_voltage;
			// PX4_INFO("Remaining battery : %f", (double) _remaining);
		}
	}
}

void
Battery::determineWarning(bool connected)
{
	if (connected) {
		// propagate warning state only if the state is higher, otherwise remain in current warning state
		if (_remaining < _emergency_thr.get() || (_warning == battery_status_s::BATTERY_WARNING_EMERGENCY)) {
			_warning = battery_status_s::BATTERY_WARNING_EMERGENCY;

		} else if (_remaining < _crit_thr.get() || (_warning == battery_status_s::BATTERY_WARNING_CRITICAL)) {
			_warning = battery_status_s::BATTERY_WARNING_CRITICAL;

		} else if (_remaining < _low_thr.get() || (_warning == battery_status_s::BATTERY_WARNING_LOW)) {
			_warning = battery_status_s::BATTERY_WARNING_LOW;
		}
	}
}

void
Battery::computeScale()
{
	const float voltage_range = (_v_charged.get() - _v_empty.get());

	// reusing capacity calculation to get single cell voltage before drop
	const float bat_v = _v_empty.get() + (voltage_range * _remaining_voltage);

	_scale = _v_charged.get() / bat_v;

	if (_scale > 1.3f) { // Allow at most 30% compensation
		_scale = 1.3f;

	} else if (!PX4_ISFINITE(_scale) || _scale < 1.f) { // Shouldn't ever be more than the power at full battery
		_scale = 1.f;
	}
}

void
Battery::computeRemainingTime(float current_a)
{
	/* Only estimate remaining time with useful in flight current measurements */
	if (_current_filtered_a > 1.f) {
		/* Initialize strongly filtered current to an estimated average consumption */
		if (_current_filtered_a_for_time < 0.f) {
			/* TODO: better initial value based on "hover current" from last flight */
			_current_filtered_a_for_time = 15.f;
		}

		/* Filter current very strong, we basically want the average consumption */
		const float weight_ct = 5e-5f;
		_current_filtered_a_for_time = (1 - weight_ct) * _current_filtered_a_for_time + weight_ct * current_a;

		/* Remaining time estimation only possible with capacity */
		if (_capacity.get() > 0.f) {
			const float remaining_capacity_mah = _remaining * _capacity.get();
			const float current_ma = _current_filtered_a_for_time * 1e3f;
			_time_remaining_s = remaining_capacity_mah / current_ma * 3600.f;
		}

	} else {
		_current_filtered_a_for_time = -1.f;
		_time_remaining_s = -1.f;
	}
}
