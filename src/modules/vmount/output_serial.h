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
 * @file output_serial.h
 * @author Beat Küng <beat-kueng@gmx.net>
 *
 */

#pragma once

#include "output.h"

#include <uORB/uORB.h>
#include <uORB/topics/sensor_bias.h>
#include <uORB/topics/vehicle_local_position.h>

namespace vmount
{


/**
 ** class OutputSerial
 *  Output to a serial device
 */
class OutputSerial : public OutputBase
{
public:
	OutputSerial(const OutputConfig &output_config);
	virtual ~OutputSerial();

	virtual int initialize();

	virtual int update(const ControlData *control_data);

	virtual void print_status();

private:

	void _handle_send_heartbeat();
	hrt_abstime _last_heartbeat_timestamp = 0;

	bool _send_packet(uint8_t *payload, int payload_len, int msg_id);

	int _serial_fd = -1;
	uint8_t _seq = 0;

	sensor_bias_s	_sensors;
	vehicle_local_position_s _vlpos;
	int _sensor_bias_sub = -1;
	int _vehicle_lpos_sub = -1;
};


} /* namespace vmount */
