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



#include "yuneec_flow.h"

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_getopt.h>
#include <px4_posix.h>
#include <px4_tasks.h>

#include <drivers/drv_hrt.h>
#include <errno.h>
#include <lib/mathlib/mathlib.h>
#include <lib/conversion/rotation.h>
#include <poll.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <parameters/param.h>
#include <termios.h>
#include <unistd.h>

#include <uORB/uORB.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/distance_sensor.h>

// Init static members
char Yuneec_Flow::_device_flow[DEVICE_ARGUMENT_MAX_LENGTH] = {};
work_s Yuneec_Flow::_work = {};
int constexpr READ_BUFFER_SIZE = 64;

Yuneec_Flow::Yuneec_Flow():
	CDev("Yuneec_Flow", FLOW_DEVICE_PATH)
{
}

Yuneec_Flow::~Yuneec_Flow()
{
	work_cancel(HPWORK, &_work);

	px4_close(_uart_fd);
}

/** @see ModuleBase */
int Yuneec_Flow::task_spawn(int argc, char *argv[])
{
	// parse the arguments
	const char *device = nullptr;
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			device = myoptarg;
			strncpy(_device_flow, device, strlen(device));

			// Fix in case of overflow
			_device_flow[sizeof(_device_flow) - 1] = '\0';
			break;
		}
	}

	// Check on required arguments
	if (device == nullptr || strlen(device) == 0) {
		print_usage("Device must be specified");
		return 1;
	}

	// Instantiate task
	Yuneec_Flow *flow = nullptr;

	if (!is_running()) {
		flow = new Yuneec_Flow();

		if (flow == nullptr) {
			PX4_ERR("failed to start flow");
			return PX4_ERROR;
		}
	}

	_object.store(flow);

	// schedule a cycle to start things
	int ret = work_queue(HPWORK, &_work, (worker_t)&Yuneec_Flow::cycle_trampoline, flow, 0);

	if (ret < 0) {
		return ret;
	}

	// Since this task runs on the work-queue rather than in a separate thread,
	// we specify the workqueue task_id
	_task_id = task_id_is_work_queue;

	// wait until task is up & running (the mode_* commands depend on it)
	if (wait_until_running() < 0) {
		return -1;
	}

	return PX4_OK;
}

/** @see ModuleBase */
int
Yuneec_Flow::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

/** @see ModuleBase */
int Yuneec_Flow::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Driver module for flow. Runs on high-priority work-queue.
### Example
The module is typically started with:
yuneec_flow start -d /dev/ttyS2
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("yuneec_flow", "driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the task");
	PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, "<device>", "Device used to talk to ESCs", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "Stop the task");
	PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Display running status of task");
	return PX4_OK;
}

void Yuneec_Flow::cycle_trampoline(void *arg)
{
	Yuneec_Flow *dev = reinterpret_cast<Yuneec_Flow *>(arg);
	dev->cycle();
}

void Yuneec_Flow::read_and_pub_data()
{
	// read data from the serial port
	uint8_t rcs_buf[READ_BUFFER_SIZE] = {};
	int newBytes = px4_read(_uart_fd, &rcs_buf[0], sizeof(rcs_buf));

	// Parse optical flow data
	for (int i = 0; i < newBytes; i++) {
		mavlink_message_t msg = {};
		mavlink_status_t status = {};
		if (mavlink_parse_char(FLOW_CHAN, rcs_buf[i], &msg, &status)) {
			if (msg.msgid == MAVLINK_MSG_ID_OPTICAL_FLOW_RAD) {
				publish_flow_messages(&msg);
			}
		}
	}
}

void Yuneec_Flow::publish_flow_messages(mavlink_message_t *msg)
{
	mavlink_optical_flow_rad_t flow = {};
	struct optical_flow_s optical_flow = {};

	/* optical flow */
	mavlink_msg_optical_flow_rad_decode(msg, &flow);

	optical_flow.timestamp = hrt_absolute_time();
	optical_flow.integration_timespan = flow.integration_time_us;
	optical_flow.pixel_flow_x_integral = -flow.integrated_x;
	optical_flow.pixel_flow_y_integral = -flow.integrated_y;
	optical_flow.gyro_x_rate_integral = -flow.integrated_xgyro;
	optical_flow.gyro_y_rate_integral = -flow.integrated_ygyro;
	optical_flow.gyro_z_rate_integral = flow.integrated_zgyro;
	optical_flow.time_since_last_sonar_update = flow.time_delta_distance_us;
	optical_flow.ground_distance_m = flow.distance ;
	optical_flow.quality = flow.quality;
	optical_flow.sensor_id = flow.sensor_id;
	optical_flow.gyro_temperature = flow.temperature;

	if (_flow_pub == nullptr) {
		_flow_pub = orb_advertise(ORB_ID(optical_flow), &optical_flow);

	} else {
		orb_publish(ORB_ID(optical_flow), _flow_pub, &optical_flow);
	}

	/* Use distance value for distance sensor topic */
	struct distance_sensor_s ground_distance = {};

	if (flow.distance > 0.0f) {  // negative values signal invalid data
		ground_distance.timestamp = hrt_absolute_time();
		ground_distance.min_distance = 0.35f;  // measurements start at 30cm, reliable after 35cm
		ground_distance.max_distance = 5.f;  //According to the specs, to determine
		ground_distance.current_distance = flow.distance;  // both are in m
		ground_distance.type = distance_sensor_s::MAV_DISTANCE_SENSOR_ULTRASOUND;
		ground_distance.id = 0;
		ground_distance.orientation = distance_sensor_s::ROTATION_DOWNWARD_FACING;
		ground_distance.covariance = 0.f;

		// There are 2 sonars -> use multi
		int orb_class_instance;

		if (_flow_distance_sensor_pub == nullptr) {
			_flow_distance_sensor_pub = orb_advertise_multi(ORB_ID(distance_sensor), &ground_distance,
						    &orb_class_instance, ORB_PRIO_LOW);

		} else {
			orb_publish(ORB_ID(distance_sensor), _flow_distance_sensor_pub, &ground_distance);
		}
	}
}

int Yuneec_Flow::init()
{
	// Init UART
	int ret = initialise_uart(_device_flow);

	if (ret < 0) {
		PX4_ERR("initialise uart failed");
		return -1;
	}

	// Init Device
	ret = CDev::init();

	if (ret != OK) {
		PX4_ERR("CDev init failed");
		return ret;
	}

	return PX4_OK;
}

void Yuneec_Flow::cycle()
{
	// Initialize module
	// For some reason UART does not work when the initialization is done sooner
	// right after the instantiation.
	if(!_initialized){
		if(init() !=0){
			PX4_ERR("Could not init module");
		}
		_initialized = true;
	}

	// Process data
	read_and_pub_data();

	if (!should_exit()) {
		// Schedule next cycle.
			work_queue(HPWORK, &_work, (worker_t)&Yuneec_Flow::cycle_trampoline, this,
				   USEC2TICK(CONVERSION_INTERVAL_FLOW));
	}
}

int Yuneec_Flow::initialise_uart(const char *device)
{
	// open uart
	_uart_fd = px4_open(device, O_RDWR | O_NONBLOCK | O_NOCTTY);

	if (_uart_fd < 0) {
		PX4_ERR("failed to open uart device!");
		return -1;
	}

	struct termios uart_config;

	int termios_state = -1;

	// fill the struct for the new configuration
	tcgetattr(_uart_fd, &uart_config);

	// Configure the terminal (see also https://en.wikibooks.org/wiki/Serial_Programming/termios )
	// clear ONLCR flag (which appends a CR for every LF)
	uart_config.c_oflag &= ~ONLCR;

	// input baud rate
	if ((termios_state = cfsetispeed(&uart_config, B500000)) < 0) {
		PX4_ERR("UART error: %d (cfsetispeed)", termios_state);
		px4_close(_uart_fd);
		return -1;
	}

	// output baud rate
	if ((termios_state = cfsetospeed(&uart_config, B500000)) < 0) {
		PX4_ERR("UART error: %d (cfsetospeed)", termios_state);
		px4_close(_uart_fd);
		return -1;
	}

	// Apply change immediately
	if ((termios_state = tcsetattr(_uart_fd, TCSANOW, &uart_config)) < 0) {
		PX4_ERR("UART error: %d (tcsetattr)", termios_state);
		px4_close(_uart_fd);
		return -1;
	}

	return PX4_OK;
}

// driver 'main' command
extern "C" __EXPORT int yuneec_flow_main(int argc, char *argv[]);
int yuneec_flow_main(int argc, char *argv[])
{
	return Yuneec_Flow::main(argc, argv);
}
