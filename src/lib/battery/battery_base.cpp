/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
 * @file battery_base.cpp
 *
 * Base class for battery estimation.
 *
 */

#include "battery_base.hpp"
#include <float.h>
#include <px4_defines.h>
#include <cstring>

// Yuneec-specific
#include <systemlib/mavlink_log.h>


void
Battery::update(hrt_abstime timestamp,
		float voltage_v,
		float current_a,
		bool connected,
		bool selected_source,
		int priority,
		float throttle_normalized,
		bool armed,
		battery_status_s *battery_status)
{

	reset(battery_status);
	battery_status->timestamp = timestamp;

	filterVoltage(voltage_v);
	filterCurrent(current_a);
	filterThrottle(throttle_normalized);

	if (_voltage_filtered_v / _n_cells.get() < 3.4f) {
		PX4_WARN("Battery is dangerously low. Land immediately to avoid deep discharge.");
	}

	updateBatteryStatus(timestamp, voltage_v, current_a, connected, selected_source, priority, throttle_normalized, armed,
			    battery_status);

	if (_battery_initialized) {
		_deltatime = (timestamp - _last_timestamp) / 1e6;

		determineWarning(connected);
	}

	battery_status->reliably_connected = true;
#ifdef BOARD_HAS_POWER_CHECK

	if (_link_check.get()) {
		battery_status->reliably_connected = !stm32_gpioread(POWER_CHECK_GPIO);
	}

#endif

	if (_voltage_filtered_v > 2.1f) {
		_battery_initialized = true;
		battery_status->voltage_v = voltage_v;
		battery_status->voltage_filtered_v = _voltage_filtered_v;
		battery_status->current_a = current_a;
		battery_status->current_filtered_a = _current_filtered_a;
		battery_status->warning = _warning;
		battery_status->remaining = _remaining;
		battery_status->connected = connected;
		battery_status->system_source = selected_source;
		battery_status->priority = priority;
		battery_status->tethered = _tethered;
		battery_status->time_remaining_s = _time_remaining_s;
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
