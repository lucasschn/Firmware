/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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



#include <px4_module.h>
#include <px4_posix.h>
#include <px4_workqueue.h>

#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <drivers/device/device.h>
#include <uORB/uORB.h>

#include <v2.0/yuneec/mavlink.h>
#include <v2.0/mavlink_types.h>

#include <uORB/topics/optical_flow.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/optical_flow_binary.h>
#include <uORB/topics/optical_flow_upgrade_mode.h>
#include <uORB/topics/optical_flow_upgrade_ack.h>

/* Measurement rate is 100 Hz for Optical Flow info and 1KHz for image upgrade */
#define CONVERSION_INTERVAL_FLOW	(1000000 / 100)	  /* microseconds, the measurement rate is 100 Hz */
#define UPGRADE_INTERVAL_FLOW	(1000000 / 1000)	  /* microseconds, the measurement rate is 1K Hz */
#define FLOW_DEVICE_PATH "/dev/flow"
#define FLOW_CHAN 1

#if !defined(DEVICE_ARGUMENT_MAX_LENGTH)
#  define DEVICE_ARGUMENT_MAX_LENGTH 20
#endif

class Flow: public device::CDev, public ModuleBase<Flow>
{
public:
	Flow();
	~Flow();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);
private:

	bool _initialized;
	static char _device_flow[DEVICE_ARGUMENT_MAX_LENGTH];

	static struct work_s	_work;
	int _uart_fd = -1;

	int	_optical_flow_binary_sub;			/**< vehicle optical upgrade setpoint */
	int	_optical_flow_upgrade_mode_sub;
	orb_advert_t _flow_pub;
	orb_advert_t _optical_flow_upgrade_ack_pub;
	orb_advert_t _flow_distance_sensor_pub;
	struct optical_flow_binary_s _optical_flow_binary;
	struct optical_flow_upgrade_mode_s _optical_flow_upgrade_mode;

	static void cycle_trampoline(void *arg);
	int init_flow();  				 	 // init - initialise the sensor
	void poll_subscriptions();	 			 // update all msg
	void cycle_flow();
	int initialise_uart(const char *device);

	void read_flow_data();
	void handle_message_optical_flow_rad(mavlink_message_t *msg);
};
