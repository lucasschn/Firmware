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
 * @file throttle_bat.hpp
 *
 * Extended Kalman Filter for battery estimation.
 *
 */

#pragma once

#include "battery_base.hpp"

class BatteryThrottle : public Battery
{
public:
	BatteryThrottle() = default;

	virtual ~BatteryThrottle() = default;

	void updateStatus(hrt_abstime timestamp,
		    float voltage_v,
		    float current_a,
		    bool connected,
		    bool selected_source,
		    int priority,
		    float throttle_normalized,
		    bool armed,
		    battery_status_s *status) override;

private:
	void sumDischarged(hrt_abstime timestamp, float current_a);
	void computeScale();
	void computeRemainingTime(float current_a);
	void estimateRemaining(float voltage_v, float current_a, float throttle, bool armed);

	bool _battery_initialized = false;
	float _voltage_filtered_v = -1.f;
	float _throttle_filtered = -1.f;
	float _current_filtered_a = -1.f;
	float _discharged_mah = 0.f;
	float _discharged_mah_loop = 0.f;
	float _remaining_voltage = -1.f;		///< normalized battery charge level remaining based on voltage
	float _remaining = -1.f;			///< normalized battery charge level, selected based on config param
	float _scale = 1.f;
	float _current_filtered_a_for_time = -1.f;
	float _time_remaining_s = -1.f;
	uint8_t _warning = battery_status_s::BATTERY_WARNING_NONE;
	hrt_abstime _last_timestamp = 0.0f;
	bool _tethered = false;
	orb_advert_t _mavlink_log_pub = nullptr;

	DEFINE_PARAMETERS_CUSTOM_PARENT(Battery,
					(ParamFloat<px4::params::BAT_V_TETHER>) _v_tether,
					(ParamFloat<px4::params::BAT_CAPACITY>) _capacity,
					(ParamFloat<px4::params::BAT_V_LOAD_DROP>) _v_load_drop,
					(ParamFloat<px4::params::BAT_R_INTERNAL>) _r_internal,
					(ParamInt<px4::params::BAT_LINK_CHECK>) _link_check)
};