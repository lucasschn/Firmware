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

#include <stdint.h>

#include <px4_defines.h>
#include <px4_module.h>
#include <px4_tasks.h>
#include <px4_getopt.h>
#include <px4_posix.h>
#include <errno.h>

#include <cmath>	// NAN
#include <cstring>

#include <lib/mathlib/mathlib.h>
#include <lib/led/led.h>
#include <lib/tunes/tunes.h>
#include <drivers/device/device.h>
#include <perf/perf_counter.h>
#include <px4_module_params.h>
#include <uORB/uORB.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/led_control.h>
#include <uORB/topics/test_motor.h>
#include <uORB/topics/tune_control.h>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/esc_status.h>
#include <uORB/topics/multirotor_motor_limits.h>
#include <uORB/topics/parameter_update.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_mixer.h>
#include <systemlib/mavlink_log.h>
#include <mixer/mixer.h>
#include <pwm_limit/pwm_limit.h>
#include "tap_esc_common.h"
#include "fault_tolerant_control/fault_tolerant_control.h"

#include "drv_tap_esc.h"

// Skip ESC ID verification on snapdragon
#ifdef __PX4_QURT
#  define BOARD_TAP_ESC_NO_VERIFY_CONFIG
#endif

#if !defined(DEVICE_ARGUMENT_MAX_LENGTH)
#  define DEVICE_ARGUMENT_MAX_LENGTH 32
#endif

// uorb update rate for control groups in miliseconds
#if !defined(TAP_ESC_CTRL_UORB_UPDATE_INTERVAL)
#  define TAP_ESC_CTRL_UORB_UPDATE_INTERVAL 2
#endif

/*
 * This driver connects to TAP ESCs via serial.
 */
class TAP_ESC : public device::CDev, public ModuleBase<TAP_ESC>, public ModuleParams
{
public:
	TAP_ESC(char const *const device, uint8_t channels_count, bool hitl);
	virtual ~TAP_ESC();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static TAP_ESC *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	virtual int init();
	virtual int ioctl(device::file_t *filp, int cmd, unsigned long arg);
	void cycle();

private:
	char 			_device[DEVICE_ARGUMENT_MAX_LENGTH];
	int 			_uart_fd = -1;
	static const uint8_t 	_device_out_map[TAP_ESC_MAX_MOTOR_NUM];
	bool 			_is_armed = false;
	int			_armed_sub = -1;
	int 			_test_motor_sub = -1;
	int 			_params_sub = -1;
	orb_advert_t        	_outputs_pub = nullptr;
	actuator_outputs_s      _outputs = {};
	actuator_armed_s	_armed = {};

	perf_counter_t	_perf_control_latency;

	int			_control_subs[actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS];
	actuator_controls_s 	_controls[actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS];
	orb_id_t		_control_topics[actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS];
	px4_pollfd_struct_t	_poll_fds[actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS];
	unsigned		_poll_fds_num = 0;

	orb_advert_t      _esc_feedback_pub = nullptr;
	orb_advert_t      _to_mixer_status = nullptr; 	///< mixer status flags
	orb_advert_t      _mavlink_log_pub = nullptr;
	esc_status_s      _esc_feedback = {};
	uint8_t    	  _channels_count = 0; 		///< nnumber of ESC channels
	uint8_t 	  _responding_esc = 0;

	MixerGroup	*_mixers = nullptr;
	uint32_t	_groups_required = 0;
	uint32_t	_groups_subscribed = 0;
	ESC_UART_BUF 	_uartbuf = {};
	EscPacket 	_packet = {};

	hrt_abstime	_wait_esc_save_log = 0;

	// Tune related members (not upstream)
	Tunes 		_tunes;
	tune_control_s 	_tune;
	hrt_abstime 	_next_tone;
	bool 		_play_tone = false;
	int 		_tune_control_sub = -1;
	inline void send_tune_packet(EscbusTunePacket &tune_packet);

	// LED related members (not upstream)
	LedControlData 	_led_control_data = {};
	LedController 	_led_controller;
	int 		_led_control_sub = -1;

	// FTC related members (not upstream)
	FaultTolerantControl 	*_fault_tolerant_control = nullptr;
	int 			_stall_by_lost_prop = -1;
	int 			esc_failure_check(uint8_t channel_id);

	// HITL related members (not upstream)
	bool 	_hitl = false;

	DEFINE_PARAMETERS(
		(ParamBool<px4::params::MC_AIRMODE>) _airmode   ///< multicopter air-mode
	)

	void subscribe();
	void send_esc_outputs(const uint16_t *pwm, const uint8_t motor_cnt);
	static int control_callback_trampoline(uintptr_t handle,
					       uint8_t control_group, uint8_t control_index, float &input);
	inline int control_callback(uint8_t control_group, uint8_t control_index, float &input);
};

const uint8_t TAP_ESC::_device_out_map[TAP_ESC_MAX_MOTOR_NUM] = ESC_OUT;

TAP_ESC::TAP_ESC(char const *const device, uint8_t channels_count, bool hitl):
	CDev("tap_esc", TAP_ESC_DEVICE_PATH),
	ModuleParams(nullptr),
	_perf_control_latency(perf_alloc(PC_ELAPSED, "tap_esc control latency")),
	_channels_count(channels_count),
	_tunes(120, 2, 4, Tunes::NoteMode::NORMAL),
	_hitl(hitl)
{
	strncpy(_device, device, sizeof(_device));
	_device[sizeof(_device) - 1] = '\0';  // Fix in case of overflow

	_control_topics[0] = ORB_ID(actuator_controls_0);
	_control_topics[1] = ORB_ID(actuator_controls_1);
	_control_topics[2] = ORB_ID(actuator_controls_2);
	_control_topics[3] = ORB_ID(actuator_controls_3);
	memset(_controls, 0, sizeof(_controls));
	memset(_poll_fds, 0, sizeof(_poll_fds));

	for (int i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS; ++i) {
		_control_subs[i] = -1;
	}

	for (size_t i = 0; i < sizeof(_outputs.output) / sizeof(_outputs.output[0]); i++) {
		_outputs.output[i] = NAN;
	}

	_outputs.noutputs = 0;

#ifdef BOARD_SUPPORTS_FTC
	int32_t ftc_enable;
	param_get(param_find("FTC_ENABLE"), &ftc_enable);

	if (ftc_enable != 0) {
		PX4_INFO("fault-tolerant-control enabled");
		_fault_tolerant_control = new FaultTolerantControl();

	} else {
		PX4_WARN("fault-tolerant-control disabled by parameter");
	}

#endif
}

TAP_ESC::~TAP_ESC()
{
	for (unsigned i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS; i++) {
		if (_control_subs[i] >= 0) {
			orb_unsubscribe(_control_subs[i]);
			_control_subs[i] = -1;
		}
	}

	orb_unsubscribe(_armed_sub);
	orb_unsubscribe(_test_motor_sub);
	orb_unsubscribe(_params_sub);
	orb_unsubscribe(_tune_control_sub);
	orb_unsubscribe(_led_control_sub);

	orb_unadvertise(_outputs_pub);
	orb_unadvertise(_esc_feedback_pub);
	orb_unadvertise(_to_mixer_status);

	tap_esc_common::deinitialise_uart(_uart_fd);

	DEVICE_LOG("stopping");

	perf_free(_perf_control_latency);

	// clean up the alternate device node
	//unregister_class_devname(PWM_OUTPUT_BASE_DEVICE_PATH, _class_instance);
	if (_fault_tolerant_control != nullptr) {
		delete _fault_tolerant_control;
		_fault_tolerant_control = nullptr;
	}
}

/** @see ModuleBase */
TAP_ESC *TAP_ESC::instantiate(int argc, char *argv[])
{
	/* Parse arguments */
	const char *device = nullptr;
	uint8_t channels_count = 0;
	bool hitl = false;

	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;

	if (argc < 2) {
		print_usage("not enough arguments");
		return nullptr;
	}

	while ((ch = px4_getopt(argc, argv, "d:n:l", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			device = myoptarg;
			break;

		case 'n':
			channels_count = atoi(myoptarg);
			break;

		case 'l':
			hitl = true;
			break;
		}
	}

	/* Sanity check on arguments */
	if (channels_count == 0) {
		print_usage("Channel count is invalid (0)");
		return nullptr;
	}

	if (device == nullptr || strlen(device) == 0) {
		print_usage("no device specified");
		return nullptr;
	}

	TAP_ESC *tap_esc = new TAP_ESC(device, channels_count, hitl);

	if (tap_esc == nullptr) {
		PX4_ERR("failed to instantiate module");
		return nullptr;
	}

	if (tap_esc->init() != 0) {
		PX4_ERR("failed to initialize module");
		delete tap_esc;
		return nullptr;
	}

	return tap_esc;
}

/** @see ModuleBase */
int TAP_ESC::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int TAP_ESC::init()
{
	int ret;

	ret = tap_esc_common::initialise_uart(_device, _uart_fd);

	if (ret != 0) {
		PX4_ERR("failed to initialise UART.");
		return ret;
	}

	/* Respect boot time required by the ESC FW */
	hrt_abstime uptime_us = hrt_absolute_time();

	if (uptime_us < MAX_BOOT_TIME_MS * 1000) {
		usleep((MAX_BOOT_TIME_MS * 1000) - uptime_us);
	}

	/* To Unlock the ESC from the Power up state we need to issue 10
	 * ESCBUS_MSG_ID_RUN request with all the values 0;
	 */
	EscPacket unlock_packet = {PACKET_HEAD, _channels_count, ESCBUS_MSG_ID_RUN};
	unlock_packet.len *= sizeof(unlock_packet.d.reqRun.rpm_flags[0]);
	memset(unlock_packet.d.bytes, 0, sizeof(unlock_packet.d.bytes));

	int unlock_times = 10;

	while (unlock_times--) {

		tap_esc_common::send_packet(_uart_fd, unlock_packet, -1);

		/* Min Packet to Packet time is 1 Ms so use 2 */
		usleep(2000);
	}

	/* do regular cdev init */
	ret = CDev::init();

	/* advertise the mixed control outputs, insist on the first group output */
	_outputs_pub = orb_advertise(ORB_ID(actuator_outputs), &_outputs);
	_esc_feedback_pub = orb_advertise(ORB_ID(esc_status), &_esc_feedback);
	multirotor_motor_limits_s multirotor_motor_limits = {};
	_to_mixer_status = orb_advertise(ORB_ID(multirotor_motor_limits), &multirotor_motor_limits);

	_armed_sub = orb_subscribe(ORB_ID(actuator_armed));
	_test_motor_sub = orb_subscribe(ORB_ID(test_motor));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_led_control_sub = orb_subscribe(ORB_ID(led_control));
	_led_controller.init(_led_control_sub);
	_tune_control_sub = orb_subscribe(ORB_ID(tune_control));

	return ret;
}

void TAP_ESC::subscribe()
{
	/* subscribe/unsubscribe to required actuator control groups */
	uint32_t sub_groups = _groups_required & ~_groups_subscribed;
	uint32_t unsub_groups = _groups_subscribed & ~_groups_required;
	_poll_fds_num = 0;

	for (unsigned i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS; i++) {
		if (sub_groups & (1 << i)) {
			DEVICE_DEBUG("subscribe to actuator_controls_%d", i);
			_control_subs[i] = orb_subscribe(_control_topics[i]);
		}

		if (unsub_groups & (1 << i)) {
			DEVICE_DEBUG("unsubscribe from actuator_controls_%d", i);
			orb_unsubscribe(_control_subs[i]);
			_control_subs[i] = -1;
		}

		if (_control_subs[i] >= 0) {
			_poll_fds[_poll_fds_num].fd = _control_subs[i];
			_poll_fds[_poll_fds_num].events = POLLIN;
			_poll_fds_num++;
		}
	}
}

void TAP_ESC::send_esc_outputs(const uint16_t *pwm, const uint8_t motor_cnt)
{
	uint16_t rpm[TAP_ESC_MAX_MOTOR_NUM] = {};
	_led_controller.update(_led_control_data);

	for (uint8_t i = 0; i < motor_cnt; i++) {
		rpm[i] = pwm[i];

		if ((rpm[i] & RUN_CHANNEL_VALUE_MASK) > RPMMAX) {
			rpm[i] = (rpm[i] & ~RUN_CHANNEL_VALUE_MASK) | RPMMAX;

		} else if ((rpm[i] & RUN_CHANNEL_VALUE_MASK) < RPMSTOPPED) {
			rpm[i] = (rpm[i] & ~RUN_CHANNEL_VALUE_MASK) | RPMSTOPPED;
		}

		// apply the led color
		if (i < BOARD_MAX_LEDS) {
			switch (_led_control_data.leds[i].color) {
			case led_control_s::COLOR_RED:
				rpm[i] |= RUN_RED_LED_ON_MASK;
				break;

			case led_control_s::COLOR_GREEN:
				rpm[i] |= RUN_GREEN_LED_ON_MASK;
				break;

			case led_control_s::COLOR_BLUE:
				rpm[i] |= RUN_BLUE_LED_ON_MASK;
				break;

			case led_control_s::COLOR_AMBER: //make it the same as yellow
			case led_control_s::COLOR_YELLOW:
				rpm[i] |= RUN_RED_LED_ON_MASK | RUN_GREEN_LED_ON_MASK;
				break;

			case led_control_s::COLOR_PURPLE:
				rpm[i] |= RUN_RED_LED_ON_MASK | RUN_BLUE_LED_ON_MASK;
				break;

			case led_control_s::COLOR_CYAN:
				rpm[i] |= RUN_GREEN_LED_ON_MASK | RUN_BLUE_LED_ON_MASK;
				break;

			case led_control_s::COLOR_WHITE:
				rpm[i] |= RUN_RED_LED_ON_MASK | RUN_GREEN_LED_ON_MASK | RUN_BLUE_LED_ON_MASK;
				break;

			default: // led_control_s::COLOR_OFF
				break;
			}
		}
	}

	rpm[_responding_esc] |= RUN_FEEDBACK_ENABLE_MASK;

	EscPacket packet = {PACKET_HEAD, _channels_count, ESCBUS_MSG_ID_RUN};
	packet.len *= sizeof(packet.d.reqRun.rpm_flags[0]);

	for (uint8_t i = 0; i < _channels_count; i++) {
		packet.d.reqRun.rpm_flags[i] = rpm[i];
	}

	int ret = tap_esc_common::send_packet(_uart_fd, packet, _responding_esc);

	if (++_responding_esc >= _channels_count) {
		_responding_esc = 0;
	}

	if (ret < 1) {
		PX4_WARN("TX ERROR: ret: %d, errno: %d", ret, errno);
	}
}

void TAP_ESC::send_tune_packet(EscbusTunePacket &tune_packet)
{
	EscPacket buzzer_packet = {PACKET_HEAD, sizeof(EscbusTunePacket), ESCBUS_MSG_ID_TUNE};
	buzzer_packet.d.tunePacket = tune_packet;
	tap_esc_common::send_packet(_uart_fd, buzzer_packet, -1);
}

int TAP_ESC::esc_failure_check(uint8_t channel_id)
{
	if (!_is_armed) {
		// clear all motor state
		_esc_feedback.engine_failure_report.motor_state = 0;

	}

	// the motor has happen failure so far it will not check esc state again when the vehicle is armed
	if (_esc_feedback.engine_failure_report.motor_state & (1 << channel_id)) {
		return channel_id;

	} else {
		// check esc feedback state is error
		if (_esc_feedback.esc[channel_id].esc_state == ESC_STATUS_ERROR_MOTOR_STALL
		    || _esc_feedback.esc[channel_id].esc_state == ESC_STATUS_ERROR_HARDWARE
		    || _esc_feedback.esc[channel_id].esc_state == ESC_STATUS_ERROR_LOSE_CMD
		    || _esc_feedback.esc[channel_id].esc_state == ESC_STATUS_ERROR_LOSE_PROPELLER) {

			// update ESC save log start time
			_wait_esc_save_log = hrt_absolute_time();

			// set this motor mask and has failure
			_esc_feedback.engine_failure_report.motor_state |= 1 << channel_id;

			// print error information for user when a motor is failing
			// Check for armed state to prevent flooding mavlink log during tests
			if (_is_armed) {
				mavlink_log_critical(&_mavlink_log_pub, "MOTOR %d ERROR IS %d",
						     channel_id, _esc_feedback.esc[channel_id].esc_state);
			}

			return channel_id;
		}
	}

	return PX4_ERROR;
}

void TAP_ESC::cycle()
{
	if (_groups_subscribed != _groups_required) {
		subscribe();
		_groups_subscribed = _groups_required;

		/* Set uorb update rate */
		for (unsigned i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS; i++) {
			if (_control_subs[i] >= 0) {
				orb_set_interval(_control_subs[i], TAP_ESC_CTRL_UORB_UPDATE_INTERVAL);
				DEVICE_DEBUG("New actuator update interval: %ums", TAP_ESC_CTRL_UORB_UPDATE_INTERVAL);
			}
		}
	}

#ifdef __PX4_QURT

	if (_groups_required == 0) {
		// This driver has no work to do, for instance when mixer has not been
		// loaded yet. There is no reason to run through the cycle now.
		return;
	}

#endif

	if (_mixers) {
		_mixers->set_airmode(_airmode.get());
	}

	/* check if anything updated.
	 * the timeout needs to be small in order to react promptly to tune requests
	 */
	int ret = px4_poll(_poll_fds, _poll_fds_num, 5);

	/* this would be bad... */
	if (ret < 0) {
		DEVICE_LOG("poll error %d", errno);

	} else { /* update even in the case of a timeout, to check for test_motor commands */

		/* get controls for required topics */
		unsigned poll_id = 0;

		for (unsigned i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS; i++) {
			if (_control_subs[i] >= 0) {
				if (_poll_fds[poll_id].revents & POLLIN) {
					orb_copy(_control_topics[i], _control_subs[i], &_controls[i]);

				}

				poll_id++;
			}
		}

		// NOTE: HOTFIX! REMOVE ASAP
		// This is a hotfix for issue #1454 (restoring FTC to original functionality)
		// We tell the mixer that FTC is running by suddenly reducing the number
		// of outputs.
		uint8_t motor_fault_count = 0;

		if (_fault_tolerant_control != nullptr) {

			/* get the fault motor counter */
			for (unsigned i = 0; i < _channels_count; i++) {
				if (_esc_feedback.engine_failure_report.motor_state & (1 << i)) {
					motor_fault_count ++;
				}
			}
		}

		/* get the actual (i.e. non-faulty) number of motors */
		uint8_t num_outputs = _channels_count - motor_fault_count;

		/* can we mix? */
		if (_is_armed && _mixers != nullptr) {

			/* do mixing */
			num_outputs = _mixers->mix(&_outputs.output[0], num_outputs);
			_outputs.noutputs = num_outputs;

			/* publish mixer status */
			multirotor_motor_limits_s multirotor_motor_limits = {};
			multirotor_motor_limits.saturation_status = _mixers->get_saturation_status();

			orb_publish(ORB_ID(multirotor_motor_limits), _to_mixer_status, &multirotor_motor_limits);

			/* disable unused ports by setting their output to NaN */
			for (size_t i = num_outputs; i < sizeof(_outputs.output) / sizeof(_outputs.output[0]); i++) {
				_outputs.output[i] = NAN;
			}

			/* iterate actuators */
			for (unsigned i = 0; i < num_outputs; i++) {
				/* last resort: catch NaN, INF and out-of-band errors */
				if (i < _outputs.noutputs && PX4_ISFINITE(_outputs.output[i])
				    && !_armed.lockdown && !_armed.manual_lockdown) {
					/* scale for PWM output 1200 - 1900us */
					_outputs.output[i] = (RPMMAX + RPMMIN) / 2 + ((RPMMAX - RPMMIN) / 2) * _outputs.output[i];
					math::constrain(_outputs.output[i], (float)RPMMIN, (float)RPMMAX);

				} else {
					/*
					 * Value is NaN, INF, or we are in lockdown - stop the motor.
					 * This will be clearly visible on the servo status and will limit the risk of accidentally
					 * spinning motors. It would be deadly in flight.
					 */
					_outputs.output[i] = RPMSTOPPED;
				}
			}

		} else {

			_outputs.noutputs = num_outputs;
			_outputs.timestamp = hrt_absolute_time();

			/* check for motor test commands */
			bool test_motor_updated;
			orb_check(_test_motor_sub, &test_motor_updated);

			if (test_motor_updated) {
				struct test_motor_s test_motor;
				orb_copy(ORB_ID(test_motor), _test_motor_sub, &test_motor);
				_outputs.output[test_motor.motor_number] = RPMSTOPPED + ((RPMMAX - RPMSTOPPED) * test_motor.value);
				PX4_INFO("setting motor %i to %.1lf", test_motor.motor_number,
					 (double)_outputs.output[test_motor.motor_number]);
			}

			/* set the invalid values to the minimum */
			for (unsigned i = 0; i < num_outputs; i++) {
				if (!PX4_ISFINITE(_outputs.output[i])) {
					_outputs.output[i] = RPMSTOPPED;
				}
			}

			/* disable unused ports by setting their output to NaN */
			for (size_t i = num_outputs; i < sizeof(_outputs.output) / sizeof(_outputs.output[0]); i++) {
				_outputs.output[i] = NAN;
			}

		}

		uint16_t motor_out[TAP_ESC_MAX_MOTOR_NUM];

		// We need to remap from the system default to what PX4's normal
		// scheme is
#ifdef BOARD_MAP_ESC_TO_PX4_OUT

		uint8_t num = 0;

		// Loop over 0 to 5 in case of hex configuration
		for (num = 0; num < num_outputs; num++) {
			motor_out[num] = _outputs.output[_device_out_map[num]];
		}

		// Loop over 6,7 in case of hex configuration
		for (num = num_outputs; num < TAP_ESC_MAX_MOTOR_NUM; num++) {
			motor_out[num] = RPMSTOPPED;
		}

#else

		switch (_channels_count) {
		case 4:
			motor_out[0] = (uint16_t)_outputs.output[2];
			motor_out[1] = (uint16_t)_outputs.output[1];
			motor_out[2] = (uint16_t)_outputs.output[0];
			motor_out[3] = (uint16_t)_outputs.output[3];
			break;

		case 6:
			motor_out[0] = (uint16_t)_outputs.output[3];
			motor_out[1] = (uint16_t)_outputs.output[0];
			motor_out[2] = (uint16_t)_outputs.output[4];
			motor_out[3] = (uint16_t)_outputs.output[2];
			motor_out[4] = (uint16_t)_outputs.output[1];
			motor_out[5] = (uint16_t)_outputs.output[5];
			break;

		default:

			// Use the system defaults
			for (uint8_t i = 0; i < _channels_count; ++i) {
				motor_out[i] = (uint16_t)_outputs.output[i];
			}

			break;
		}

#endif

		if (_fault_tolerant_control != nullptr) {

			int failure_motor_num = -1;

			for (uint8_t channel_id = 0; channel_id < _channels_count; channel_id++) {
				failure_motor_num = esc_failure_check(channel_id);

				// enter fault tolerant control
				if (failure_motor_num == channel_id) {

					// update fault tolerant controller parameters about PID or others, only for debug PID parameters
					_fault_tolerant_control->parameter_update_poll();

					// find the motor with the failure motor is diagonal
					uint8_t diagonal_motor_num = 0;

					// TODO: to distribute the vehicle motor number of QUAD_X ,this is only for vehicle is HEX_X now
					// failure motor number: 0 1 2 3 4 5
					// diagonal motor number:3 4 5 0 1 2
					if (failure_motor_num + 3 > 5) {
						diagonal_motor_num = failure_motor_num - 3;

					} else {
						diagonal_motor_num = failure_motor_num + 3;
					}

					// check the diagonal motor is failure,will stop it.
					if (esc_failure_check(diagonal_motor_num) == diagonal_motor_num) {

						// wait ESC save log time,because ESC save log frequency is 5Hz.if we stop motor esc state will clear
						if (((hrt_absolute_time() - _wait_esc_save_log) > 5000000)
						    || (_esc_feedback.esc[diagonal_motor_num].esc_setpoint_raw == RPMSTOPPED)) {
							// stop the failure motor
							motor_out[diagonal_motor_num] = RPMSTOPPED;

						}

					} else {
						// recalculate output of the motor with the failure motor is diagonal
						motor_out[diagonal_motor_num] = _fault_tolerant_control->recalculate_pwm_outputs(motor_out[failure_motor_num],
										motor_out[diagonal_motor_num], _esc_feedback.engine_failure_report.delta_pwm);

						// wait ESC save log time,because ESC save log frequency is 5Hz.if we stop motor esc state will clear
						if (((hrt_absolute_time() - _wait_esc_save_log) > 5000000)
						    || (_esc_feedback.esc[failure_motor_num].esc_setpoint_raw == RPMSTOPPED)) {
							// stop the failure motor
							motor_out[failure_motor_num] = RPMSTOPPED;

						}
					}

					// check a motor stall is caused by a collision of another motor's lost propeller
					if (_esc_feedback.esc[failure_motor_num].esc_state == ESC_STATUS_ERROR_MOTOR_STALL) {
						// check whether the other motor is lost propeller
						for (uint8_t lose_id = 0; lose_id < _channels_count; lose_id++) {
							if (_esc_feedback.esc[lose_id].esc_state == ESC_STATUS_ERROR_LOSE_PROPELLER) {
								// stop the failure motor try restart this motor
								motor_out[failure_motor_num] = RPMSTOPPED;
								// set the flag when the motor stall by a collision of another motor's lost propeller
								_stall_by_lost_prop = failure_motor_num;
								break;
							}
						}

					}

					// when ESC unlock to clear motor failure mask
					if ((hrt_absolute_time() - _wait_esc_save_log) > 50000 && (failure_motor_num == _stall_by_lost_prop)) {
						// clear this motor mask and has failure
						_esc_feedback.engine_failure_report.motor_state &= ~(1 << failure_motor_num);
						_stall_by_lost_prop = -1;
					}
				}
			}


		} else {
			// Even without FTC we want to check for motor failures and report them
			for (uint8_t channel_id = 0; channel_id < _channels_count; channel_id++) {
				esc_failure_check(channel_id);
			}
		}

		// Kill switch is enabled, emergency stop. Also in HITL the motors should never turn
		if (_armed.manual_lockdown || _hitl) {
			for (unsigned i = 0; i < num_outputs; ++i) {
				motor_out[i] = RPMSTOPPED;
			}
		}

		_outputs.timestamp = hrt_absolute_time();

		send_esc_outputs(motor_out, _channels_count);
		tap_esc_common::read_data_from_uart(_uart_fd, &_uartbuf);

		if (tap_esc_common::parse_tap_esc_feedback(&_uartbuf, &_packet) == 0) {
			if (_packet.msg_id == ESCBUS_MSG_ID_RUN_INFO) {
				RunInfoRepsonse &feed_back_data = _packet.d.rspRunInfo;

				if (feed_back_data.channelID < esc_status_s::CONNECTED_ESC_MAX) {
					_esc_feedback.esc[feed_back_data.channelID].esc_rpm = feed_back_data.speed;
#ifdef ESC_HAVE_VOLTAGE_SENSOR
					_esc_feedback.esc[feed_back_data.channelID].esc_voltage = feed_back_data.voltage;
#endif
#ifdef ESC_HAVE_CURRENT_SENSOR
					// ESCs report in 10mA/LSB
					_esc_feedback.esc[feed_back_data.channelID].esc_current = feed_back_data.current / 100.f;
#endif
#ifdef ESC_HAVE_TEMPERATURE_SENSOR
					_esc_feedback.esc[feed_back_data.channelID].esc_temperature = feed_back_data.temperature;
#endif
					_esc_feedback.esc[feed_back_data.channelID].esc_state = feed_back_data.ESCStatus;
					_esc_feedback.esc[feed_back_data.channelID].esc_vendor = esc_status_s::ESC_VENDOR_TAP;
					_esc_feedback.esc[feed_back_data.channelID].esc_setpoint_raw = motor_out[feed_back_data.channelID];
					// PWM convert to RPM,PWM:1200~1900<-->RPM:1600~7500 so rpm = 1600 + (pwm - 1200)*((7500-1600)/(1900-1200))
					_esc_feedback.esc[feed_back_data.channelID].esc_setpoint = (float)motor_out[feed_back_data.channelID] * 8.43f - 8514.3f;
					_esc_feedback.esc_connectiontype = esc_status_s::ESC_CONNECTION_TYPE_SERIAL;
					_esc_feedback.counter++;
					_esc_feedback.esc_count = _channels_count;

					_esc_feedback.timestamp = hrt_absolute_time();

					// Don't publish ESC feedback in HITL mode. Reading and parsing the
					// feedback is still done (see above) such that the hardware load
					// is as close as possible to when actually flying without HITL.
					if (!_hitl) {
						orb_publish(ORB_ID(esc_status), _esc_feedback_pub, &_esc_feedback);
					}
				}
			}
		}

		/* and publish for anyone that cares to see */
		if (!_hitl) {
			orb_publish(ORB_ID(actuator_outputs), _outputs_pub, &_outputs);
		}

		// use first valid timestamp_sample for latency tracking
		for (int i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS; i++) {
			const bool required = _groups_required & (1 << i);
			const hrt_abstime &timestamp_sample = _controls[i].timestamp_sample;

			if (required && (timestamp_sample > 0)) {
				perf_set_elapsed(_perf_control_latency, _outputs.timestamp - timestamp_sample);
				break;
			}
		}

	}

	bool updated;

	orb_check(_armed_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(actuator_armed), _armed_sub, &_armed);

		if (_is_armed != _armed.armed) {
			/* reset all outputs */
			for (size_t i = 0; i < sizeof(_outputs.output) / sizeof(_outputs.output[0]); i++) {
				_outputs.output[i] = NAN;
			}
		}

		_is_armed = _armed.armed;

	}

	// Handle tunes
	updated = false;
	orb_check(_tune_control_sub, &updated);
	hrt_abstime now = hrt_absolute_time();

	if (updated) {
		orb_copy(ORB_ID(tune_control), _tune_control_sub, &_tune);

		if (_tunes.set_control(_tune) == 0) {
			_next_tone = hrt_absolute_time();
			_play_tone = true;

		} else {
			_play_tone = false;
		}
	}

	unsigned frequency = 0, duration = 0, silence = 0;
	uint8_t strength = 0;
	EscbusTunePacket esc_tune_packet;

	if ((now >= _next_tone) && _play_tone) {
		_play_tone = false;
		int parse_ret_val = _tunes.get_next_tune(frequency, duration, silence, strength);

		// Is there right now a tone that needs to be played?
		// the return value is 0 if one tone need to be played and 1 if the sequence needs to continue
		if (parse_ret_val >= 0 && frequency > 0 && duration > 0 && strength > 0) {
			esc_tune_packet.frequency = frequency;
			esc_tune_packet.duration_ms = (uint16_t)(duration / 1000); // convert to ms
			esc_tune_packet.strength = (_hitl ? tune_control_s::STRENGTH_HITL : strength);
			// set next tone call time
			_next_tone = now + silence + duration;

			if (!_is_armed || _armed.manual_lockdown) {
				send_tune_packet(esc_tune_packet);
			}
		}

		// Does a tone follow after this one?
		if (parse_ret_val > 0) {
			_play_tone = true;
		}
	}

	/* check for parameter updates */
	bool param_updated = false;
	orb_check(_params_sub, &param_updated);

	if (param_updated) {
		struct parameter_update_s update;
		orb_copy(ORB_ID(parameter_update), _params_sub, &update);
		updateParams();
	}
}

int TAP_ESC::control_callback_trampoline(uintptr_t handle, uint8_t control_group, uint8_t control_index, float &input)
{
	TAP_ESC *obj = (TAP_ESC *)handle;
	return obj->control_callback(control_group, control_index, input);
}

int TAP_ESC::control_callback(uint8_t control_group, uint8_t control_index, float &input)
{
	input = _controls[control_group].control[control_index];

	/* limit control input */
	if (input > 1.0f) {
		input = 1.0f;

	} else if (input < -1.0f) {
		input = -1.0f;
	}

	/* throttle not arming - mark throttle input as invalid */
	if (_armed.prearmed && !_armed.armed) {
		if ((control_group == actuator_controls_s::GROUP_INDEX_ATTITUDE ||
		     control_group == actuator_controls_s::GROUP_INDEX_ATTITUDE_ALTERNATE) &&
		    control_index == actuator_controls_s::INDEX_THROTTLE) {
			/* set the throttle to an invalid value */
			input = NAN;
		}
	}

	return 0;
}

int TAP_ESC::ioctl(device::file_t *filp, int cmd, unsigned long arg)
{
	int ret = OK;

	switch (cmd) {

	case MIXERIOCRESET:
		if (_mixers != nullptr) {
			delete _mixers;
			_mixers = nullptr;
			_groups_required = 0;
		}

		break;

	case MIXERIOCLOADBUF: {
			const char *buf = (const char *)arg;
			unsigned buflen = strlen(buf);

			if (_mixers == nullptr) {
				_mixers = new MixerGroup(control_callback_trampoline, (uintptr_t)this);
			}

			if (_mixers == nullptr) {
				_groups_required = 0;
				ret = -ENOMEM;

			} else {

				ret = _mixers->load_from_buf(buf, buflen);

				if (ret != 0) {
					DEVICE_DEBUG("mixer load failed with %d", ret);
					delete _mixers;
					_mixers = nullptr;
					_groups_required = 0;
					ret = -EINVAL;

				} else {

					_mixers->groups_required(_groups_required);
				}
			}

			break;
		}

	default:
		ret = -ENOTTY;
		break;
	}

	return ret;
}

/** @see ModuleBase */
void TAP_ESC::run()
{
	// Main loop
	while (!should_exit()) {
		cycle();
	}
}

/** @see ModuleBase */
int TAP_ESC::task_spawn(int argc, char *argv[])
{
	/* start the task */
	_task_id = px4_task_spawn_cmd("tap_esc",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_ACTUATOR_OUTPUTS,
				      1300,
				      (px4_main_t)&run_trampoline,
				      argv);

	if (_task_id < 0) {
		PX4_ERR("task start failed");
		_task_id = -1;
		return PX4_ERROR;
	}

	// wait until task is up & running
	if (wait_until_running() < 0) {
		_task_id = -1;
		return -1;
	}

	return PX4_OK;
}

/** @see ModuleBase */
int TAP_ESC::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This module controls the ESCs as well as the LEDs via UART. It listens on the
actuator_controls topics, does the mixing and writes the PWM outputs.

Additionally it offers commands for flashing the ESC firmware.

### Implementation
Currently the module is implementd as a threaded version only, meaning that it
runs in its own thread instead of on the work queue.

### Example
The module is typically started with:
tap_esc start -d /dev/ttyS2 -n <1-8>

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("tap_esc", "driver");

	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the task");
	PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, "<device>", "Device used to talk to ESCs", true);
	PRINT_MODULE_USAGE_PARAM_INT('n', 4, 0, 8, "Number of ESCs", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('l',"HITL (hardware in the loop), no motor outputs and esc feedback are sent", true);

	PRINT_MODULE_USAGE_COMMAND_DESCR("checkcrc", "deprecated. Use `tap_esc_config` instead");
	PRINT_MODULE_USAGE_COMMAND_DESCR("upload", "deprecated. Use `tap_esc_config` instead");
	PRINT_MODULE_USAGE_COMMAND_DESCR("config", "deprecated. Use `tap_esc_config` instead");
	return PX4_OK;
}

extern "C" __EXPORT int tap_esc_main(int argc, char *argv[]);

int tap_esc_main(int argc, char *argv[])
{
	return TAP_ESC::main(argc, argv);
}
