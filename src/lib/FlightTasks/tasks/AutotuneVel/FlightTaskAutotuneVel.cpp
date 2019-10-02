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
 * @file FlightTaskAutotunevel.cpp
 */

#include "FlightTaskAutotuneVel.hpp"
#include <float.h>
#include <mathlib/mathlib.h>

using namespace matrix;

bool FlightTaskAutotuneVel::initializeSubscriptions(SubscriptionArray &subscription_array)
{
	if (!FlightTaskManual::initializeSubscriptions(subscription_array)) {
		return false;
	}

	return true;
}

bool FlightTaskAutotuneVel::updateInitialize()
{
	bool ret = FlightTaskManual::updateInitialize();
	// require valid position / velocity in xy
	return ret && PX4_ISFINITE(_position(0))
	       && PX4_ISFINITE(_position(1))
	       && PX4_ISFINITE(_velocity(0))
	       && PX4_ISFINITE(_velocity(1));
}

bool FlightTaskAutotuneVel::activate()
{
	bool ret = FlightTaskManual::activate();
	_yaw_setpoint = NAN;
	_yawspeed_setpoint = 0.0f;
	_thrust_setpoint = matrix::Vector3f(0.0f, 0.0f, NAN);
	_position_setpoint(2) = _position(2);
	_velocity_setpoint(2) = 0.0f;
	_done = false;

	return ret;
}

bool FlightTaskAutotuneVel::_checkSticks()
{
	// Abort if the user moves the sticks
	if (Vector3f(&_sticks(0)).length() > 0.2f) {
		return true;

	} else {
		return false;
	}
}

void FlightTaskAutotuneVel::_applystep()
{
	start_step_time = hrt_absolute_time()/1e6;
	_thrust_setpoint(0) = 0.5;
	_thrust_setpoint(1) = 0.f;
	_thrust_setpoint(2) = NAN;
	if (_velocity(0) < prev_velocity - epsilon || _velocity(0) > prev_velocity + epsilon){

		stable_count ++;
	}

	if (stable_count > 1.5f){
		_thrust_setpoint(0) = 0.f;
		_done = true;
	}
}

void FlightTaskAutotuneVel::_measureoutput()
{
	time = hrt_absolute_time()/1e6;
	a = _velocity(0) - prev_velocity / time - prev_time;
	if (a>maxa){
		maxa = a;
		velocity_a = _velocity(0);
		time_elapsed = time - start_step_time;
		// velocity_maxa - time_elapsed*a = -aL
		L = time_elapsed - velocity_a/a;
	}

	prev_velocity = _velocity(0);
	prev_time = time;
}

void FlightTaskAutotuneVel::_computeControlGains()
{
	// Compute Kp, Ki and Kd using robust Ziegler-Nichols rules
	float kp = 1.2f/(maxa*L);
	float ki = 2.f*L;
	float kd = 0.5f*L;
	printf("Kp = %.3f\tKi =%.3f\tKd = %.3f\n", (double)kp, (double)ki, (double)kd);

	if (kp > 0.f && kp < 1.f && ki > 0.f && ki < 4.f && kd > 0.f && kd < 0.1f) {
		_param_mpc_xy_vel_p.set(kp);
		_param_mpc_xy_vel_p.commit_no_notification();
		_param_mpc_xy_vel_i.set(ki);
		_param_mpc_xy_vel_i.commit_no_notification();
		_param_mpc_xy_vel_d.set(kd);
		_param_mpc_xy_vel_d.commit_no_notification();

	} else {
		printf("Autotuning failed");
	}
}

void FlightTaskAutotuneVel::_exit()
{
	_param_mpc_xy_vel_atune.set(false);
	_param_mpc_xy_vel_atune.commit();
	_done = true;
}

bool FlightTaskAutotuneVel::update()
{
	if (_done) {
		return false;
	}

	if (_checkSticks()) {
		_exit();
		return false;
	}

	_applystep();
	_measureoutput();

	if (_done) {
		_computeControlGains();
	}

	return true;
}
