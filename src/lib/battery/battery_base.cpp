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
Battery::filter1order(float &signal_filtered, const float signal_raw, const float alpha)
{

	const float filtered = signal_filtered * (1.f - alpha) + signal_raw * alpha;

	if (PX4_ISFINITE(filtered)) {
		signal_filtered = filtered;
	}
}

void
Battery::determineWarning(uint8_t &warning, const bool &connected, const float &remaining)
{
	if (connected) {
		// propagate warning state only if the state is higher, otherwise remain in current warning state
		if (remaining < _emergency_thr.get() || (warning == battery_status_s::BATTERY_WARNING_EMERGENCY)) {
			warning = battery_status_s::BATTERY_WARNING_EMERGENCY;

		} else if (remaining < _crit_thr.get() || (warning == battery_status_s::BATTERY_WARNING_CRITICAL)) {
			warning = battery_status_s::BATTERY_WARNING_CRITICAL;

		} else if (remaining < _low_thr.get() || (warning == battery_status_s::BATTERY_WARNING_LOW)) {
			warning = battery_status_s::BATTERY_WARNING_LOW;
		}
	}
}
