/****************************************************************************
 *
 *   Copyright (c) 2015, 2016 Airmind Development Team. All rights reserved.
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
 * 3. Neither the name Airmind nor the names of its contributors may be
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
 * @file hc_sr04.cpp
 *
 * Driver for the hc_sr04 sonar range finders .
 *
 * N.B. This driver needs work to be more gneric. It is tied to
 * group 1:ch0 for trigger (channel 1) and group 1:ch1 (channel 2)
 * for echo.
 *
 * It expects the fmu to be started with fmu mode_pwm3 and will
 * set the alt rate of 20 Hz and then set up capture on channel 2
 */

#include <px4_config.h>
#include <px4_workqueue.h>
#include <drivers/device/device.h>
#include <px4_defines.h>
#include <containers/Array.hpp>
#include <px4_log.h>
#include <platforms/px4_getopt.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <getopt.h>

#include <perf/perf_counter.h>
#include <systemlib/px4_macros.h>
#include <systemlib/err.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_range_finder.h>
#include <drivers/device/ringbuffer.h>

#include <controllib/blocks.hpp>

#include <uORB/uORB.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/manual_control_setpoint.h>

#include <lib/conversion/rotation.h>

#include <board_config.h>
#include <drivers/drv_input_capture.h>
#include <drivers/drv_pwm_output.h>

#include <DevMgr.hpp>

/* Configuration Constants */
#define SR04_DEVICE_PATH	"/dev/hc_sr04"

#define SUBSYSTEM_TYPE_RANGEFINDER 131072
/* Device limits */
#define SR04_MIN_DISTANCE 	(0.40f)
#define SR04_MAX_DISTANCE 	(5.00f)
#define SR04_TIME_2_DISTANCE_M (0.000170f)

#define SR04_CONVERSION_INTERVAL 	50000 /* 50ms for one sonar */

#define _MF_WINDOW_SIZE 3 ///< window size for median filter on sonar range


#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif

#define PX4FMU_DEVICE_PATH	"/dev/px4fmu" // TODO(YUNEEC): This looks wrong

using namespace DriverFramework;

int static cmp(const void *a, const void *b)
{
	return (*(const float *)a > *(const float *)b);
}

class HC_SR04 : public cdev::CDev
{
public:
	HC_SR04(uint8_t rotation, bool enable_median_filter, bool enable_obsavoid_switch);
	virtual ~HC_SR04();

	virtual int 		init();
	virtual int			ioctl(device::file_t *filp, int cmd, unsigned long arg);

	/**
	* Diagnostics - print some basic information about the driver.
	*/
	void				print_info();

protected:
	virtual int			probe();

private:

#if defined(HC_SR04_CAPTURE_DEBUG)

	typedef struct timer_inf_t {
		uint32_t chan_index;
		hrt_abstime edge_time;
		uint32_t edge_state;
		uint32_t overflow;
	} timer_inf_t;

	volatile timer_inf_t _edges[16] = {{0, 0, 0, 0}}; // Must be a power of 2
	volatile int 		 _edge_index = 0;

	volatile hrt_abstime _distance_time[16] = {0}; // Must be a power of 2
	volatile int 		 _distance_time_index_on = 0;
	volatile int 		 _distance_time_index_off = 0;
#else
	volatile hrt_abstime _distance_time = 0;
#endif

	enum _thread_state_t {
		thread_stopped = -1,
		thread_run = 0,
		thread_exit = 1,
		thread_exited = 2,
	} _thread_state = thread_stopped;

	float				_min_distance;
	float				_max_distance;
	float 				_mf_window[_MF_WINDOW_SIZE] {};
	float				_mf_window_sorted[_MF_WINDOW_SIZE] {};
	int 				_mf_cycle_counter;
	work_s				_work{};
	ringbuffer::RingBuffer	*_reports;
	volatile enum {
		Uninitialized = 0,
		Initialized,
		WatingRising,
		WatingFalling,
	} _sensor_state;

	int					_measure_ticks;
	volatile bool				_sensor_data_available;
	int					_class_instance;
	int					_orb_class_instance;

	bool				_enable_median_filter;
	bool 				_enable_obsavoid_switch;

	uint8_t 		_rotation;
	orb_advert_t		_distance_sensor_topic;

	perf_counter_t		_sample_perf;
	perf_counter_t		_comms_errors;
	perf_counter_t		_buffer_overflows;


	px4::Array<float, 6>
	_latest_sonar_measurements; /* vector to store latest sonar measurements in before writing to report */

	int 				_manual_sub;
	bool		 		_pwm_output_active;

	/**
	* Test whether the device supported by the driver is present at a
	* specific address.
	*
	* @param address	The I2C bus address to probe.
	* @return			True if the device is present.
	*/
	int					probe_address(uint8_t address);

	/**
	* Initialise the automatic measurement state machine and start it.
	*
	* @note This function is called at open and error time.  It might make sense
	*       to make it more aggressive about resetting the bus in case of errors.
	*/
	void				start();

	/**
	* Stop the automatic measurement state machine.
	*/
	void				stop();

	/**
	* Pwm configuration used at driver init and whenever the sensor is restarted through the obstacle
	* avoidance swicth in the remote controller
	*/

	int 				start_pwm();

	/**
	* Pwm configuration used when shutting down the driver and whenever the sensor is switched off
	* through the obstacle avoidance switch in the remote controller
	*/
	void				stop_pwm();

	/**
	* Set the min and max distance thresholds if you want the end points of the sensors
	* range to be brought in at all, otherwise it will use the defaults MB12XX_MIN_DISTANCE
	* and MB12XX_MAX_DISTANCE
	*/
	void				set_minimum_distance(float min);
	void				set_maximum_distance(float max);
	float				get_minimum_distance();
	float				get_maximum_distance();

	/**
	* Perform a poll cycle; collect from the previous measurement
	* and start a new one.
	*/
	void				cycle();
	/**
	* Static trampoline from the workq context; because we don't have a
	* generic workq wrapper yet.
	*
	* @param arg		Instance pointer for the driver that is polling.
	*/
	static void			cycle_trampoline(void *arg);

	static void capture_trampoline(void *context, uint32_t chan_index,
				       hrt_abstime edge_time, uint32_t edge_state, uint32_t overflow);

	void capture_callback(uint32_t chan_index,
			      hrt_abstime edge_time, uint32_t edge_state, uint32_t overflow);

	float median_filter(float value);
	void  reset_median_filter();


};


/*
 * Driver 'main' command.
 */
extern "C"  __EXPORT int hc_sr04_main(int argc, char *argv[]);

HC_SR04::HC_SR04(uint8_t rotation, bool enable_median_filter, bool enable_obsavoid_switch) :
	CDev(SR04_DEVICE_PATH),
	_min_distance(SR04_MIN_DISTANCE),
	_max_distance(SR04_MAX_DISTANCE),
	_mf_cycle_counter(0),
	_reports(nullptr),
	_sensor_state(Uninitialized),
	_measure_ticks(0),
	_sensor_data_available(false),
	_class_instance(-1),
	_orb_class_instance(-1),
	_enable_median_filter(enable_median_filter),
	_enable_obsavoid_switch(enable_obsavoid_switch),
	_rotation(rotation),
	_distance_sensor_topic(nullptr),
	_sample_perf(perf_alloc(PC_ELAPSED, "hc_sr04_read")),
	_comms_errors(perf_alloc(PC_COUNT, "hc_sr04_comms_errors")),
	_buffer_overflows(perf_alloc(PC_COUNT, "hc_sr04_buffer_overflows")),
	_manual_sub(-1),
	_pwm_output_active(false)

{
}

HC_SR04::~HC_SR04()
{
	/* make sure we are truly inactive */
	stop();

	/* free any existing reports */
	if (_reports != nullptr) {
		delete _reports;
	}

	if (_class_instance != -1) {
		unregister_class_devname(RANGE_FINDER_BASE_DEVICE_PATH, _class_instance);
	}

	/* free perf counters */
	perf_free(_sample_perf);
	perf_free(_comms_errors);
	perf_free(_buffer_overflows);
}

int
HC_SR04::init()
{
	/* do init (and probe) first */
	if (CDev::init() != OK) {
		return PX4_ERROR;
	}

	/* allocate basic report buffers */
	_reports = new ringbuffer::RingBuffer(2, sizeof(distance_sensor_s));

	if (_reports == nullptr) {
		return PX4_ERROR;
	}

	_class_instance = register_class_devname(RANGE_FINDER_BASE_DEVICE_PATH);

	int fd_fmu = ::open(PX4FMU_DEVICE_PATH, O_RDWR);

	if (fd_fmu == -1) {
		PX4_ERR("FMU: open fail");
		return PX4_ERROR;
	}

	/* Set PWM rate for the sonar to 20Hz (other groups will be 50) */

	if (::ioctl(fd_fmu, PWM_SERVO_SET_UPDATE_RATE, 20) != OK) {
		PX4_ERR("PWM_SERVO_SET_UPDATE_RATE fail");
		::close(fd_fmu);
		return PX4_ERROR;
	}

	unsigned group = 1;
	uint32_t group_mask;

	/* Get the group mask */

	if (::ioctl(fd_fmu, PWM_SERVO_GET_RATEGROUP(group), (unsigned long)&group_mask) != OK) {
		PX4_ERR("PWM_SERVO_GET_RATEGROUP group %u fail", group);
		::close(fd_fmu);
		return PX4_ERROR;
	}

	/* Apply rate on group mask */

	if (::ioctl(fd_fmu, PWM_SERVO_SET_SELECT_UPDATE_RATE, group_mask) != OK) {
		PX4_ERR("PWM_SERVO_SET_SELECT_UPDATE_RATE fail");
		::close(fd_fmu);
		return PX4_ERROR;
	}

	/* Now Update the PWM rates and start the PWM (but with 0 width pulses) */

	if (::ioctl(fd_fmu, PWM_SERVO_ARM, 0) != OK) {
		PX4_ERR("PWM_SERVO_ARM fail");
		::close(fd_fmu);
		return PX4_ERROR;
	}

	/* Now that the rates are set. Reclaim the capture pin
	 * fm the PWM by switching the fmu mode to PWM2CAP2 without a
	 * rate change.
	 */

	if (::ioctl(fd_fmu, INPUT_CAP_SET_COUNT, 2) != 0) {
		PX4_ERR("INPUT_CAP_SET_COUNT fail");
		return PX4_ERROR;
	}

	input_capture_config_t cap_config;
	cap_config.channel = 2;
	cap_config.filter = 0xf;
	cap_config.edge = Both;
	cap_config.callback = &HC_SR04::capture_trampoline;
	cap_config.context = this; // pass reference to this class

	/* Now set up the capture */

	if (::ioctl(fd_fmu, INPUT_CAP_SET, (unsigned long)&cap_config) != 0) {
		PX4_ERR("INPUT_CAP_SET fail");
		return PX4_ERROR;
	}

	::close(fd_fmu);

	/* Start the PWM pulses if an obsavoid switch will NOT be used */

	if (!_enable_obsavoid_switch) {
		if (start_pwm() < 0) {
			PX4_ERR("Not able to start pwm.");
			return PX4_ERROR;
		}
	}

	/* sensor is ok, but we don't really know if it is within range */
	_sensor_state = Initialized;

	// start driver

	start();

	return OK;
}

int
HC_SR04::probe()
{
	return (OK);
}

void
HC_SR04::set_minimum_distance(float min)
{
	_min_distance = min;
}

void
HC_SR04::set_maximum_distance(float max)
{
	_max_distance = max;
}

float
HC_SR04::get_minimum_distance()
{
	return _min_distance;
}

float
HC_SR04::get_maximum_distance()
{
	return _max_distance;
}

// TODO: review this part since it is not compatible with the new architecture
int
HC_SR04::ioctl(device::file_t *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	case SENSORIOCSPOLLRATE: {
			switch (arg) {

			/* zero would be bad */
			case 0:
				return -EINVAL;

			/* set default polling rate */
			case SENSOR_POLLRATE_DEFAULT: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_ticks == 0);

					/* set interval for next measurement to minimum legal value */
					_measure_ticks = USEC2TICK(SR04_CONVERSION_INTERVAL);

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();

					}

					return OK;
				}

			/* adjust to a legal polling interval in Hz */
			default: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_ticks == 0);

					/* convert hz to tick interval via microseconds */
					int ticks = USEC2TICK(1000000 / arg);

					/* check against maximum rate */
					if (ticks < USEC2TICK(SR04_CONVERSION_INTERVAL)) {
						return -EINVAL;
					}

					/* update interval for next measurement */
					_measure_ticks = ticks;

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();
					}

					return OK;
				}
			}
		}

	default:
		/* give it to the superclass */
		return CDev::ioctl(filp, cmd, arg);
	}
}

void
HC_SR04::reset_median_filter(void)
{
	for (int i = 0; i < _MF_WINDOW_SIZE; ++i) {
		_mf_window[i] = get_maximum_distance();
	}

	_mf_cycle_counter = 0;
}

float
HC_SR04::median_filter(float value)
{
	/* TODO: replace with ring buffer */
	_mf_window[(_mf_cycle_counter + 1) % _MF_WINDOW_SIZE] = value;

	for (int i = 0; i < _MF_WINDOW_SIZE; ++i) {
		_mf_window_sorted[i] = _mf_window[i];
	}

	qsort(_mf_window_sorted, _MF_WINDOW_SIZE, sizeof(float), cmp);

	_mf_cycle_counter++;

	if (_mf_window_sorted[(_MF_WINDOW_SIZE / 2)] < get_maximum_distance()) {
		return _mf_window_sorted[(_MF_WINDOW_SIZE / 2)];

	} else {
		return get_maximum_distance();
	}
}

void
HC_SR04::start()
{
	/* reset the report ring */
	_reports->flush();

	_thread_state = thread_run;
	work_queue(HPWORK, &_work, (worker_t)&HC_SR04::cycle_trampoline, this, 0);
}

void
HC_SR04::stop()
{
	// Stop the worker

	if (_thread_state == thread_run) {
		_thread_state = thread_exit;

		// insure it is stopped
		int wait = ((SR04_CONVERSION_INTERVAL + 10000) / 1000);

		while (_thread_state != thread_exited && wait--) {
			usleep(1000);
		}
	}

	// Stop the pwm and hence the capture

	stop_pwm();


	int fd_fmu = ::open(PX4FMU_DEVICE_PATH, O_RDWR);

	if (fd_fmu == -1) {
		PX4_WARN("FMU: px4_open fail\n");
	}

	input_capture_config_t cap_config;
	cap_config.channel = 2;
	cap_config.filter = 0;
	cap_config.edge = Disabled;
	cap_config.callback = nullptr;
	cap_config.context = nullptr;

	::ioctl(fd_fmu, INPUT_CAP_SET_CALLBACK, (unsigned long)&cap_config);
	::close(fd_fmu);

	/* unadvertise publishing topics */
	orb_unadvertise(_distance_sensor_topic);
}

int
HC_SR04::start_pwm()
{
	int rv = PX4_ERROR;
	int fd_pwm = ::open(PWM_OUTPUT0_DEVICE_PATH, O_RDWR);

	if (fd_pwm < 0) {
		PX4_ERR("PMW: open fail");

	} else {

		// Set pulsewidth of 10ms specific for this sonar sensor

		rv = ::ioctl(fd_pwm, PWM_SERVO_SET(1), 10);

		if (rv >= 0) {
			_pwm_output_active = true;

		} else {
			PX4_ERR("PWM_SERVO_SET channel 1 failed.");
		}
	}

	::close(fd_pwm);
	return rv;
}

void
HC_SR04::stop_pwm()
{

	int fd_pwm = ::open(PWM_OUTPUT0_DEVICE_PATH, O_RDWR);

	if (fd_pwm >= 0) {
		if (::ioctl(fd_pwm, PWM_SERVO_SET(1), 0) >= 0) {
			_pwm_output_active = false;
			// reset median filter window if the pwm is stopped
			reset_median_filter();

		} else {
			PX4_ERR("PWM_SERVO_SET channel 1 failed.");
		}
	}

	::close(fd_pwm);
}

void
HC_SR04::cycle_trampoline(void *arg)
{
	HC_SR04 *dev = (HC_SR04 *)arg;
	dev->cycle();

}

void
HC_SR04::cycle()
{
	if (_thread_state == thread_exit) {
		_thread_state = thread_exited;
		return;
	}

	/* check obstacle avoidance switch position in the remote controller:
	* - if avoidance enabled (SWITCH_POS_ON and SWITCH_POS_MIDDLE) the ultrasonic sensor is on
	* - else the ultrasonic sensor is switched off
	*/
	if (_enable_obsavoid_switch) {
		if (_manual_sub == -1) {
			_manual_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
		}

		bool updated;
		orb_check(_manual_sub, &updated);

		if (updated) {
			struct manual_control_setpoint_s manual;
			orb_copy(ORB_ID(manual_control_setpoint), _manual_sub, &manual);

			if (manual.obsavoid_switch == manual_control_setpoint_s::SWITCH_POS_ON ||
			    manual.obsavoid_switch == manual_control_setpoint_s::SWITCH_POS_MIDDLE) {

				/* The set point message is published periodically; We only want the deltas
				 * to cause changes in the physical PWM state.
				 */

				if (!_pwm_output_active) {
					start_pwm();
				}

			} else {
				if (_pwm_output_active) {
					stop_pwm();
				}
			}
		}
	}

	// publish measured distance only if ultrasonic sensor is on
	if (_pwm_output_active) {

		irqstate_t flags = px4_enter_critical_section();
		bool sensor_data_available = _sensor_data_available;

		// reset flag
		if (_sensor_data_available) {
			_sensor_data_available = false;
		}

		float distance = _distance_time * SR04_TIME_2_DISTANCE_M;
		px4_leave_critical_section(flags);

		if (!sensor_data_available) {
			work_queue(HPWORK, &_work, (worker_t)&HC_SR04::cycle_trampoline, this, USEC2TICK(SR04_CONVERSION_INTERVAL));
			return;
		}

		struct distance_sensor_s report = {};

		report.timestamp = hrt_absolute_time();

		report.min_distance = get_minimum_distance();

		report.max_distance = get_maximum_distance();

		if (_enable_median_filter) {
			report.current_distance = median_filter(distance);

		} else {
			report.current_distance = distance;
		}

		report.type = distance_sensor_s::MAV_DISTANCE_SENSOR_ULTRASOUND;
		report.orientation = _rotation;

		/* publish it, if we are the primary */
		if (_distance_sensor_topic != nullptr) {
			orb_publish(ORB_ID(distance_sensor), _distance_sensor_topic, &report);

		} else {
			_distance_sensor_topic = orb_advertise_multi(ORB_ID(distance_sensor), &report,
						 &_orb_class_instance, ORB_PRIO_LOW);
		}
	}

	work_queue(HPWORK, &_work, (worker_t)&HC_SR04::cycle_trampoline, this, USEC2TICK(SR04_CONVERSION_INTERVAL));
}

void
HC_SR04::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	perf_print_counter(_buffer_overflows);
	printf("poll interval:  %u ticks\n", _measure_ticks);
	_reports->print_info("report queue");
}

void HC_SR04::capture_trampoline(void *context, uint32_t chan_index,
				 hrt_abstime edge_time, uint32_t edge_state, uint32_t overflow)
{
	HC_SR04 *dev = reinterpret_cast<HC_SR04 *>(context);
	dev->capture_callback(chan_index, edge_time, edge_state, overflow);
}

void HC_SR04::capture_callback(uint32_t chan_index,
			       hrt_abstime edge_time, uint32_t edge_state, uint32_t overflow)
{
	static hrt_abstime raising_time = 0;
	static hrt_abstime falling_time = 0;

#if defined(HC_SR04_CAPTURE_DEBUG)
	_edges[_edge_index].chan_index = chan_index;
	_edges[_edge_index].edge_time  = edge_time;
	_edges[_edge_index].edge_state =  edge_state;
	_edges[_edge_index++].overflow = overflow;
	_edge_index &= arraySize(_edges) - 1;
#endif

	if (edge_state == 1) {
		_sensor_state = WatingFalling;
		raising_time = edge_time;

	} else {
		_sensor_state = WatingRising;
		falling_time = edge_time;

#if defined(HC_SR04_CAPTURE_DEBUG)
		_distance_time[_distance_time_index_on++] = falling_time - raising_time;
		_distance_time_index_on &= arraySize(_distance_time) - 1;
#else
		_distance_time = falling_time - raising_time;
#endif
		_sensor_data_available = true;
	}
}

/**
 * Local functions in support of the shell command.
 */
namespace  hc_sr04
{

HC_SR04	*g_dev;

void	start(uint8_t rotation, bool enable_median_filter, bool enable_obsavoid_switch);
void	stop();
void	test();
void	info();
void	reset();

/**
 * Start the driver.
 */
void
start(uint8_t rotation, bool enable_median_filter, bool enable_obsavoid_switch)
{
	if (g_dev != nullptr) {
		PX4_ERR("already started");
		exit(1);
	}

	/* create the driver */
	g_dev = new HC_SR04(rotation, enable_median_filter, enable_obsavoid_switch);

	if (g_dev == nullptr) {
		goto fail;
	}

	if (OK != g_dev->init()) {
		goto fail;
	}

	exit(0);

fail:

	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	PX4_ERR("driver start failed");
	exit(1);
}

/**
 * Stop the driver
 */
void stop()
{
	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;

	} else {
		PX4_ERR("driver not running");
		exit(1);
	}

	exit(0);
}

// TODO: need reimplementation
/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
void
test()
{
#if 0
	struct distance_sensor_s report;
	ssize_t sz;
	int ret;

	int fd = open(SR04_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		err(1, "%s open failed (try 'hc_sr04 start' if the driver is not running", SR04_DEVICE_PATH);
	}

	/* do a simple demand read */
	sz = read(fd, &report, sizeof(report));

	if (sz != sizeof(report)) {
		err(1, "immediate read failed");
	}

	print_message(report);

	/* start the sensor polling at 2Hz */
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, 2)) {
		errx(1, "failed to set 2Hz poll rate");
	}

	/* read the sensor 5x and report each value */
	for (unsigned i = 0; i < 5; i++) {
		struct pollfd fds;

		/* wait for data to be ready */
		fds.fd = fd;
		fds.events = POLLIN;
		ret = poll(&fds, 1, 2000);

		if (ret != 1) {
			errx(1, "timed out waiting for sensor data");
		}

		/* now go get it */
		sz = read(fd, &report, sizeof(report));

		if (sz != sizeof(report)) {
			err(1, "periodic read failed");
		}

		print_message(report);
	}

	/* reset the sensor polling to default rate */
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT)) {
		errx(1, "failed to set default poll rate");
	}

	errx(0, "PASS");
#endif
}

/**
 * Reset the driver.
 */
void
reset()
{
	int fd = open(SR04_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		err(1, "failed ");
	}

	if (ioctl(fd, SENSORIOCRESET, 0) < 0) {
		err(1, "driver reset failed");
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		err(1, "driver poll restart failed");
	}

	exit(0);
}

/**
 * Print a little info about the driver.
 */
void
info()
{
	if (g_dev == nullptr) {
		PX4_ERR("driver not running");
		exit(1);
	}

	printf("state @ %p\n", g_dev);
	g_dev->print_info();

	exit(0);
}

} /* namespace */


int
hc_sr04_main(int argc, char *argv[])
{

	int ch;
	bool enable_median_filter = false;
	bool enable_obsavoid_switch = false;
	uint8_t rotation = distance_sensor_s::ROTATION_FORWARD_FACING;
	int myoptind = 1;
	const char *myoptarg = NULL;


	while ((ch = px4_getopt(argc, argv, "R:ma", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'R':
			rotation = (uint8_t)atoi(myoptarg);
			break;

		case 'm':
			enable_median_filter = true;
			break;

		case 'a':
			enable_obsavoid_switch = true;
			break;

		default:
			PX4_WARN("missing rotation information");
		}
	}

	const char *verb = argv[myoptind];

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(verb, "start")) {
		hc_sr04::start(rotation, enable_median_filter, enable_obsavoid_switch);
	}

	/*
	 * Stop the driver
	 */
	if (!strcmp(verb, "stop")) {
		hc_sr04::stop();
	}

	/*
	 * Test the driver/device.
	 */
	// if (!strcmp(verb, "test")) {
	// 	hc_sr04::test();
	// }

	/*
	 * Print driver information.
	 */
	if (!strcmp(verb, "info") || !strcmp(verb, "status")) {
		hc_sr04::info();
	}

	PX4_ERR("unrecognized command, try 'start', 'test' or 'info'");
	exit(1);
}
