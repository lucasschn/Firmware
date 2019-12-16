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
 * @file ekf_bat.hpp
 *
 * Extended Kalman Filter for battery estimation.
 *
 */

#pragma once

#include "battery_base.hpp"
#include <matrix/matrix/math.hpp>

class BatteryEKF : public Battery
{
public:
	BatteryEKF() = default;

	virtual ~BatteryEKF() = default;

	void updateStatus(hrt_abstime timestamp,
			  float voltage_v,
			  float current_a,
			  bool connected,
			  bool selected_source,
			  int priority,
			  float throttle_normalized,
			  bool armed,
			  battery_status_s *battery_status) override;

private:

	bool init(hrt_abstime timestamp, float voltage, float current);
	void kfInit(float voltage_v);
	void kfUpdate(hrt_abstime timestamp, float current_a, float voltage_v);
	float getSlope(float SOC);
	float ocv_from_soc(float SOC);
	float soc_from_ocv(float OCV);
	void recompute_statespace(float dt);

	// For the state of charge initialization (points for interpolation)
	float _SOC_array[4] = {0.0f, 0.04f, 0.6f, 1.0f};
	float _OCV_array[4] = {3.297f, 3.629464f, 3.90068f, 4.342f};

	// for the Kalman filter
	bool _init_kf = true;
	float _deltatime = 0.1f;
	hrt_abstime _last_timestamp = 0; //us
	float _SOC0;
	float _u = 0.0f; // input (=current)
	float _y = 0.0f; // output (=voltage)
	float _capacity_mAh = 6500; // mAh battery capacity
	float _iR10; // battery is considered at equilibrium at bootup
	bool _battery_initialized = false;
	float _voltage_filtered_v = -1.f;
	float _current_filtered_a = -1.f;

	// 2x2 identity matrix
	matrix::SquareMatrix<float, 2> I = matrix::eye<float, 2>();

	// Definition of A matrix
	matrix::SquareMatrix<float, 2> _batmat_A;

	// Definition of B vector
	matrix::Vector2f _batmat_B{_deltatime / (_capacity_mAh * 3.6f), 1.0f - (float) exp(-_deltatime / (_R1.get() * _C1.get()))}; //3.6 converts from mAh to C
	// Defintion of C matrix
	matrix::Matrix<float, 1, 2> _batmat_C;

	// Definition of D matrix
	// float const D = -R0;
	// Definition of xhat vector
	matrix::Vector2f _xhat;
	// Definition of covx matrix (needs tuning)
	// should be initialized diagonal, diagonal elements should represent confidence interval of xhat0
	matrix::SquareMatrix<float, 2> _covx;

	// Definition of covxy vector
	matrix::Vector2f covxy;
	// Definition of covw matrix (needs tuning)
	// represents inaccuracy of A and B (plus process noise)
	matrix::SquareMatrix<float, 2> _covw;

	// Definition of covv scalar (needs tuning)
	float const _covv = 0.5017f + 0.5f;
	// 0.5017 is sensor noise, but covv also represents C and D inaccuracies, so should be larger than that.
	// Definition of Kalman gain matrix L
	matrix::Matrix<float, 2, 1> _kalman_gain;
	float _yhat = 4.34f; // value only used as start value in simulation
	float _innovation;
	float _covy;

	// Definition of the coefficient of the 11th-degree polynomial defining the OCV(SOC) curve. Coeffs are ordered in descending degree.
	float _poly_coeff[12] = {1.54079666e+04, -8.79957879e+04, 2.19186396e+05, -3.12687981e+05, 2.82109975e+05, -1.67743633e+05, 6.64413360e+04, -1.73142475e+04, 2.86290965e+03, -2.80825943e+02, 1.48923600e+01, 3.33948616e+00};

	DEFINE_PARAMETERS_CUSTOM_PARENT(Battery,
					(ParamFloat<px4::params::BAT_PARAM_R0>) _R0,
					(ParamFloat<px4::params::BAT_PARAM_R1>) _R1,
					(ParamFloat<px4::params::BAT_PARAM_C1>) _C1
				       )


};
