/****************************************************************************
 *
 *	Copyright (c) 2017 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *	notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *	notice, this list of conditions and the following disclaimer in
 *	the documentation and/or other materials provided with the
 *	distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *	used to endorse or promote products derived from this software
 *	without specific prior written permission.
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

#pragma once

#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_status_flags.h>
#include <uORB/topics/led_control.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/safety.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/led_control.h>
#include "dsm.h"

/**
 * helper class to handle binding for the st24 (and enter binding mode when vehicle is upside down)
 */
class St24Helper
{
public:

	void init(int rcs_fd);

	/** send a bind command to the receiver */
	void bind();

	/** loop iteration */
	void cycle(bool rc_lost, bool armed);

	/** get the vehicle_land_detected uORB subscription */
	int vehicle_land_detected_sub() const { return _vehicle_landed_sub; }
	bool vehicle_land_detected_updated() const { return _vehicle_landed_updated; }

private:
	int     	_vehicle_landed_sub = -1;
	int     	_vehicle_status_sub = -1;
	orb_advert_t    _led_control_pub = nullptr;
	bool        	_was_inverted = false;
	int             _rcs_fd;
	bool            _vehicle_landed_updated = false;

	enum class BindingState {
		INIT = 0,
		START,
		WAIT,
		END,
		BOUND
	};

	BindingState _binding_state = BindingState::INIT;
};
