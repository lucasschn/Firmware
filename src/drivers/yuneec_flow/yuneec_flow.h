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
#include <px4_workqueue.h>

#include <drivers/device/device.h>

#include <uORB/Publication.hpp>

#include <v2.0/yuneec/mavlink.h>
#include <v2.0/mavlink_types.h>

/* Measurement rate is 100 Hz for Optical Flow info and 1KHz for image upgrade */
#define CONVERSION_INTERVAL_FLOW	(1000000 / 100)	  /* microseconds, the measurement rate is 100 Hz */
#define FLOW_DEVICE_PATH "/dev/flow"
#define FLOW_CHAN 1

#if !defined(DEVICE_ARGUMENT_MAX_LENGTH)
#  define DEVICE_ARGUMENT_MAX_LENGTH 20
#endif

class Yuneec_Flow: public device::CDev, public ModuleBase<Yuneec_Flow>
{
public:
	Yuneec_Flow();
	~Yuneec_Flow();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

private:
	static char _device_flow[DEVICE_ARGUMENT_MAX_LENGTH];
	static struct work_s	_work;
	int _uart_fd = -1;
	bool _initialized = false;

	orb_advert_t _flow_pub = nullptr;
	orb_advert_t _flow_distance_sensor_pub = nullptr;

	static void cycle_trampoline(void *arg);
	int init();
	void cycle();
	int initialise_uart(const char *device);
	void read_and_pub_data();
	void publish_flow_messages(mavlink_message_t *msg);
};
