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
 * @file rc_update.cpp
 *
 * @author Beat Kueng <beat-kueng@gmx.net>
 */

#include "rc_update.h"

#include <string.h>
#include <float.h>
#include <errno.h>

#include <uORB/uORB.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/input_rc.h>

using namespace sensors;

RCUpdate::RCUpdate(const Parameters &parameters)
	: _parameters(parameters),
	  _filter_roll(50.0f, 10.f), /* get replaced by parameter */
	  _filter_pitch(50.0f, 10.f),
	  _filter_yaw(50.0f, 10.f),
	  _filter_throttle(50.0f, 10.f)
{
	// iniitalized subscription
	for (int &sub : _rc_subs) {
		sub = -1;
	}

	memset(&_rc, 0, sizeof(_rc));
	memset(&_rc_parameter_map, 0, sizeof(_rc_parameter_map));
	memset(&_param_rc_values, 0, sizeof(_param_rc_values));
	memset(&_manual_sp, 0, sizeof(_manual_sp));
}

int RCUpdate::init()
{
	bool init_success = false;

	for (int i = 0; i < ORB_MULTI_MAX_INSTANCES; i++) {
		_rc_subs[i] = orb_subscribe_multi(ORB_ID(input_rc), i);

		if (_rc_subs[i] >= 0) {
			init_success = true;
		}
	}

	_rc_parameter_map_sub = orb_subscribe(ORB_ID(rc_parameter_map));

	if (_rc_parameter_map_sub < 0) {
		init_success = false;
	}

	if (!init_success) {
		return -errno;
	}

	return 0;
}

void RCUpdate::deinit()
{
	for (int i = 0; i < ORB_MULTI_MAX_INSTANCES; i++) {
		orb_unsubscribe(_rc_subs[i]);
		_rc_subs[i] = -1;
	}

	orb_unsubscribe(_rc_parameter_map_sub);
}

void RCUpdate::update_rc_functions()
{
	/* update RC function mappings */
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_THROTTLE] = _parameters.rc_map_throttle - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_ROLL] = _parameters.rc_map_roll - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_PITCH] = _parameters.rc_map_pitch - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_YAW] = _parameters.rc_map_yaw - 1;
	math::convertRcMode(_parameters.rc_mode,
			    _rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_ROLL],
			    _rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_PITCH],
			    _rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_YAW],
			    _rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_THROTTLE]);

	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_MODE] = _parameters.rc_map_mode_sw - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_RETURN] = _parameters.rc_map_return_sw - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_RATTITUDE] = _parameters.rc_map_rattitude_sw - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_POSCTL] = _parameters.rc_map_posctl_sw - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_LOITER] = _parameters.rc_map_loiter_sw - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_ACRO] = _parameters.rc_map_acro_sw - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_OFFBOARD] = _parameters.rc_map_offboard_sw - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_KILLSWITCH] = _parameters.rc_map_kill_sw - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_ARMSWITCH] = _parameters.rc_map_arm_sw - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_TRANSITION] = _parameters.rc_map_trans_sw - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_GEAR] = _parameters.rc_map_gear_sw - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_STAB] = _parameters.rc_map_stab_sw - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_MAN] = _parameters.rc_map_man_sw - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_OBSAVOIDSWITCH] = _parameters.rc_map_obsavoid_sw - 1;

	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_FLAPS] = _parameters.rc_map_flaps - 1;

	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_AUX_1] = _parameters.rc_map_aux1 - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_AUX_2] = _parameters.rc_map_aux2 - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_AUX_3] = _parameters.rc_map_aux3 - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_AUX_4] = _parameters.rc_map_aux4 - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_AUX_5] = _parameters.rc_map_aux5 - 1;

	for (int i = 0; i < rc_parameter_map_s::RC_PARAM_MAP_NCHAN; i++) {
		_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_PARAM_1 + i] = _parameters.rc_map_param[i] - 1;
	}

	/* update the RC low pass filter frequencies */
	_filter_roll.set_cutoff_frequency(_parameters.rc_flt_smp_rate, _parameters.rc_flt_cutoff);
	_filter_pitch.set_cutoff_frequency(_parameters.rc_flt_smp_rate, _parameters.rc_flt_cutoff);
	_filter_yaw.set_cutoff_frequency(_parameters.rc_flt_smp_rate, _parameters.rc_flt_cutoff);
	_filter_throttle.set_cutoff_frequency(_parameters.rc_flt_smp_rate, _parameters.rc_flt_cutoff);
	_filter_roll.reset(0.f);
	_filter_pitch.reset(0.f);
	_filter_yaw.reset(0.f);
	_filter_throttle.reset(0.f);
}

void
RCUpdate::rc_parameter_map_poll(ParameterHandles &parameter_handles, bool forced)
{
	bool map_updated;
	orb_check(_rc_parameter_map_sub, &map_updated);

	if (map_updated) {
		orb_copy(ORB_ID(rc_parameter_map), _rc_parameter_map_sub, &_rc_parameter_map);

		/* update parameter handles to which the RC channels are mapped */
		for (int i = 0; i < rc_parameter_map_s::RC_PARAM_MAP_NCHAN; i++) {
			if (_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_PARAM_1 + i] < 0 || !_rc_parameter_map.valid[i]) {
				/* This RC channel is not mapped to a RC-Parameter Channel (e.g. RC_MAP_PARAM1 == 0)
				 * or no request to map this channel to a param has been sent via mavlink
				 */
				continue;
			}

			/* Set the handle by index if the index is set, otherwise use the id */
			if (_rc_parameter_map.param_index[i] >= 0) {
				parameter_handles.rc_param[i] = param_for_used_index((unsigned)_rc_parameter_map.param_index[i]);

			} else {
				parameter_handles.rc_param[i] = param_find(&_rc_parameter_map.param_id[i * (rc_parameter_map_s::PARAM_ID_LEN + 1)]);
			}

		}

		PX4_DEBUG("rc to parameter map updated");

		for (int i = 0; i < rc_parameter_map_s::RC_PARAM_MAP_NCHAN; i++) {
			PX4_DEBUG("\ti %d param_id %s scale %.3f value0 %.3f, min %.3f, max %.3f",
				  i,
				  &_rc_parameter_map.param_id[i * (rc_parameter_map_s::PARAM_ID_LEN + 1)],
				  (double)_rc_parameter_map.scale[i],
				  (double)_rc_parameter_map.value0[i],
				  (double)_rc_parameter_map.value_min[i],
				  (double)_rc_parameter_map.value_max[i]
				 );
		}
	}
}

float
RCUpdate::get_rc_value(uint8_t func, float min_value, float max_value)
{
	if (_rc.function[func] >= 0) {
		float value = _rc.channels[_rc.function[func]];
		return math::constrain(value, min_value, max_value);

	} else {
		return 0.0f;
	}
}

switch_pos_t
RCUpdate::get_rc_sw3pos_position(uint8_t func, float on_th, bool on_inv, float mid_th, bool mid_inv)
{
	if (_rc.function[func] >= 0) {
		float value = 0.5f * _rc.channels[_rc.function[func]] + 0.5f;

		if (on_inv ? value < on_th : value > on_th) {
			return manual_control_setpoint_s::SWITCH_POS_ON;

		} else if (mid_inv ? value < mid_th : value > mid_th) {
			return manual_control_setpoint_s::SWITCH_POS_MIDDLE;

		} else {
			return manual_control_setpoint_s::SWITCH_POS_OFF;
		}

	} else {
		return manual_control_setpoint_s::SWITCH_POS_NONE;
	}
}

switch_pos_t
RCUpdate::get_rc_sw2pos_position(uint8_t func, float on_th, bool on_inv)
{
	if (_rc.function[func] >= 0) {
		float value = 0.5f * _rc.channels[_rc.function[func]] + 0.5f;

		if (on_inv ? value < on_th : value > on_th) {
			return manual_control_setpoint_s::SWITCH_POS_ON;

		} else {
			return manual_control_setpoint_s::SWITCH_POS_OFF;
		}

	} else {
		return manual_control_setpoint_s::SWITCH_POS_NONE;
	}
}

void
RCUpdate::set_params_from_rc(const ParameterHandles &parameter_handles)
{
	for (int i = 0; i < rc_parameter_map_s::RC_PARAM_MAP_NCHAN; i++) {
		if (_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_PARAM_1 + i] < 0 || !_rc_parameter_map.valid[i]) {
			/* This RC channel is not mapped to a RC-Parameter Channel (e.g. RC_MAP_PARAM1 == 0)
			 * or no request to map this channel to a param has been sent via mavlink
			 */
			continue;
		}

		float rc_val = get_rc_value((rc_channels_s::RC_CHANNELS_FUNCTION_PARAM_1 + i), -1.0, 1.0);

		/* Check if the value has changed,
		 * maybe we need to introduce a more aggressive limit here */
		if (rc_val > _param_rc_values[i] + FLT_EPSILON || rc_val < _param_rc_values[i] - FLT_EPSILON) {
			_param_rc_values[i] = rc_val;
			float param_val = math::constrain(
						  _rc_parameter_map.value0[i] + _rc_parameter_map.scale[i] * rc_val,
						  _rc_parameter_map.value_min[i], _rc_parameter_map.value_max[i]);
			param_set(parameter_handles.rc_param[i], &param_val);
		}
	}
}

void
RCUpdate::rc_poll(const ParameterHandles &parameter_handles)
{
	bool rc_updated = false;

	for (int i = 0; i < ORB_MULTI_MAX_INSTANCES; i++) {

		// check if subscriber has updated
		orb_check(_rc_subs[i], &rc_updated);

		// copy message into struct
		if (rc_updated) {
			orb_copy(ORB_ID(input_rc), _rc_subs[i], &_inputs_rc[i]);

			// get priority
			int32_t priority;
			orb_priority(_rc_subs[i], &priority);

			// normal RC mode
			if (_parameters.rc_link_mode == 0 && priority == ORB_PRIO_DEFAULT) {
				// map rc based on mapping function
				map_from_rc_channel_functions(_inputs_rc[i], parameter_handles);

			}
		}
	}
}

void
RCUpdate::map_from_rc_channel_functions(input_rc_s &rc_input, const ParameterHandles &parameter_handles)
{
	/* detect RC signal loss */
	bool signal_lost;

	/* check flags and require at least four channels to consider the signal valid */
	if (rc_input.rc_lost || rc_input.rc_failsafe || rc_input.channel_count < 4) {
		/* signal is lost or no enough channels */
		signal_lost = true;

	} else {
		/* signal looks good */
		signal_lost = false;

		/* check failsafe */
		int8_t fs_ch = _rc.function[_parameters.rc_map_failsafe]; // get channel mapped to throttle

		if (_parameters.rc_map_failsafe > 0) { // if not 0, use channel number instead of rc.function mapping
			fs_ch = _parameters.rc_map_failsafe - 1;
		}

		if (_parameters.rc_fails_thr > 0 && fs_ch >= 0) {
			/* failsafe configured */
			if ((_parameters.rc_fails_thr < _parameters.min[fs_ch] && rc_input.values[fs_ch] < _parameters.rc_fails_thr) ||
			    (_parameters.rc_fails_thr > _parameters.max[fs_ch] && rc_input.values[fs_ch] > _parameters.rc_fails_thr)) {
				/* failsafe triggered, signal is lost by receiver */
				signal_lost = true;
			}
		}
	}

	unsigned channel_limit = rc_input.channel_count;

	if (channel_limit > RC_MAX_CHAN_COUNT) {
		channel_limit = RC_MAX_CHAN_COUNT;
	}

	/* read out and scale values from raw message even if signal is invalid */
	for (unsigned int i = 0; i < channel_limit; i++) {

		/*
		 * 1) Constrain to min/max values, as later processing depends on bounds.
		 */
		if (rc_input.values[i] < _parameters.min[i]) {
			rc_input.values[i] = _parameters.min[i];
		}

		if (rc_input.values[i] > _parameters.max[i]) {
			rc_input.values[i] = _parameters.max[i];
		}

		/*
		 * 2) Scale around the mid point differently for lower and upper range.
		 *
		 * This is necessary as they don't share the same endpoints and slope.
		 *
		 * First normalize to 0..1 range with correct sign (below or above center),
		 * the total range is 2 (-1..1).
		 * If center (trim) == min, scale to 0..1, if center (trim) == max,
		 * scale to -1..0.
		 *
		 * As the min and max bounds were enforced in step 1), division by zero
		 * cannot occur, as for the case of center == min or center == max the if
		 * statement is mutually exclusive with the arithmetic NaN case.
		 *
		 * DO NOT REMOVE OR ALTER STEP 1!
		 */
		if (rc_input.values[i] > (_parameters.trim[i] + _parameters.dz[i])) {
			_rc.channels[i] = (rc_input.values[i] - _parameters.trim[i] - _parameters.dz[i]) / (float)(
						  _parameters.max[i] - _parameters.trim[i] - _parameters.dz[i]);

		} else if (rc_input.values[i] < (_parameters.trim[i] - _parameters.dz[i])) {
			_rc.channels[i] = (rc_input.values[i] - _parameters.trim[i] + _parameters.dz[i]) / (float)(
						  _parameters.trim[i] - _parameters.min[i] - _parameters.dz[i]);

		} else {
			/* in the configured dead zone, output zero */
			_rc.channels[i] = 0.0f;
		}

		_rc.channels[i] *= _parameters.rev[i];

		/* handle any parameter-induced blowups */
		if (!PX4_ISFINITE(_rc.channels[i])) {
			_rc.channels[i] = 0.0f;
		}
	}

	_rc.channel_count = rc_input.channel_count;
	_rc.rssi = rc_input.rssi;
	_rc.signal_lost = signal_lost;
	_rc.timestamp = rc_input.timestamp_last_signal;
	_rc.frame_drop_count = rc_input.rc_lost_frame_count;

	/* publish rc_channels topic even if signal is invalid, for debug */
	publish_rc_channels();

	/* only publish manual control if the signal is still present and was present once */
	if (!signal_lost && rc_input.timestamp_last_signal > 0) {

		/* initialize manual setpoint */
		struct manual_control_setpoint_s manual = {};
		/* set mode slot to unassigned */
		_manual_sp.mode_slot = manual_control_setpoint_s::MODE_SLOT_NONE;
		/* set the timestamp to the last signal time */
		_manual_sp.timestamp = rc_input.timestamp_last_signal;
		_manual_sp.data_source = manual_control_setpoint_s::SOURCE_RC;

		/* limit controls */
		_manual_sp.y = get_rc_value(rc_channels_s::RC_CHANNELS_FUNCTION_ROLL, -1.0, 1.0);
		_manual_sp.x = get_rc_value(rc_channels_s::RC_CHANNELS_FUNCTION_PITCH, -1.0, 1.0);
		_manual_sp.r = get_rc_value(rc_channels_s::RC_CHANNELS_FUNCTION_YAW, -1.0, 1.0);
		_manual_sp.z = (get_rc_value(rc_channels_s::RC_CHANNELS_FUNCTION_THROTTLE, -1.0, 1.0) + 1.f) / 2.f;
		_manual_sp.flaps = get_rc_value(rc_channels_s::RC_CHANNELS_FUNCTION_FLAPS, -1.0, 1.0);
		_manual_sp.aux1 = get_rc_value(rc_channels_s::RC_CHANNELS_FUNCTION_AUX_1, -1.0, 1.0);
		_manual_sp.aux2 = get_rc_value(rc_channels_s::RC_CHANNELS_FUNCTION_AUX_2, -1.0, 1.0);
		_manual_sp.aux3 = get_rc_value(rc_channels_s::RC_CHANNELS_FUNCTION_AUX_3, -1.0, 1.0);
		_manual_sp.aux4 = get_rc_value(rc_channels_s::RC_CHANNELS_FUNCTION_AUX_4, -1.0, 1.0);
		_manual_sp.aux5 = get_rc_value(rc_channels_s::RC_CHANNELS_FUNCTION_AUX_5, -1.0, 1.0);

		/* filter controls */
		_manual_sp.y = math::constrain(_filter_roll.apply(manual.y), -1.f, 1.f);
		_manual_sp.x = math::constrain(_filter_pitch.apply(manual.x), -1.f, 1.f);
		_manual_sp.r = math::constrain(_filter_yaw.apply(manual.r), -1.f, 1.f);
		_manual_sp.z = math::constrain(_filter_throttle.apply(manual.z), 0.f, 1.f);

		if (_parameters.rc_map_flightmode > 0) {

			/* the number of valid slots equals the index of the max marker minus one */
			const int num_slots = manual_control_setpoint_s::MODE_SLOT_MAX;

			/* the half width of the range of a slot is the total range
			 * divided by the number of slots, again divided by two
			 */
			const float slot_width_half = 2.0f / num_slots / 2.0f;

			/* min is -1, max is +1, range is 2. We offset below min and max */
			const float slot_min = -1.0f - 0.05f;
			const float slot_max = 1.0f + 0.05f;

			/* the slot gets mapped by first normalizing into a 0..1 interval using min
			 * and max. Then the right slot is obtained by multiplying with the number of
			 * slots. And finally we add half a slot width to ensure that integer rounding
			 * will take us to the correct final index.
			 */
			_manual_sp.mode_slot = (((((_rc.channels[_parameters.rc_map_flightmode - 1] - slot_min) * num_slots) +
						  slot_width_half) /
						 (slot_max - slot_min)) + (1.0f / num_slots));

			if (_manual_sp.mode_slot >= num_slots) {
				_manual_sp.mode_slot = num_slots - 1;
			}
		}

		/* mode switches */
		_manual_sp.mode_switch = get_rc_sw3pos_position(rc_channels_s::RC_CHANNELS_FUNCTION_MODE, _parameters.rc_auto_th,
					 _parameters.rc_auto_inv, _parameters.rc_assist_th, _parameters.rc_assist_inv);
		_manual_sp.rattitude_switch = get_rc_sw2pos_position(rc_channels_s::RC_CHANNELS_FUNCTION_RATTITUDE,
					      _parameters.rc_rattitude_th,
					      _parameters.rc_rattitude_inv);
		_manual_sp.posctl_switch = get_rc_sw2pos_position(rc_channels_s::RC_CHANNELS_FUNCTION_POSCTL, _parameters.rc_posctl_th,
					   _parameters.rc_posctl_inv);
		_manual_sp.return_switch = get_rc_sw2pos_position(rc_channels_s::RC_CHANNELS_FUNCTION_RETURN, _parameters.rc_return_th,
					   _parameters.rc_return_inv);
		_manual_sp.loiter_switch = get_rc_sw2pos_position(rc_channels_s::RC_CHANNELS_FUNCTION_LOITER, _parameters.rc_loiter_th,
					   _parameters.rc_loiter_inv);
		_manual_sp.acro_switch = get_rc_sw2pos_position(rc_channels_s::RC_CHANNELS_FUNCTION_ACRO, _parameters.rc_acro_th,
					 _parameters.rc_acro_inv);
		_manual_sp.offboard_switch = get_rc_sw2pos_position(rc_channels_s::RC_CHANNELS_FUNCTION_OFFBOARD,
					     _parameters.rc_offboard_th, _parameters.rc_offboard_inv);
		_manual_sp.kill_switch = get_rc_sw2pos_position(rc_channels_s::RC_CHANNELS_FUNCTION_KILLSWITCH,
					 _parameters.rc_killswitch_th, _parameters.rc_killswitch_inv);
		_manual_sp.arm_switch = get_rc_sw2pos_position(rc_channels_s::RC_CHANNELS_FUNCTION_ARMSWITCH,
					_parameters.rc_armswitch_th, _parameters.rc_armswitch_inv);
		_manual_sp.transition_switch = get_rc_sw2pos_position(rc_channels_s::RC_CHANNELS_FUNCTION_TRANSITION,
					       _parameters.rc_trans_th, _parameters.rc_trans_inv);
		_manual_sp.gear_switch = get_rc_sw2pos_position(rc_channels_s::RC_CHANNELS_FUNCTION_GEAR,
					 _parameters.rc_gear_th, _parameters.rc_gear_inv);
		_manual_sp.stab_switch = get_rc_sw2pos_position(rc_channels_s::RC_CHANNELS_FUNCTION_STAB,
					 _parameters.rc_stab_th, _parameters.rc_stab_inv);
		_manual_sp.man_switch = get_rc_sw2pos_position(rc_channels_s::RC_CHANNELS_FUNCTION_MAN,
					_parameters.rc_man_th, _parameters.rc_man_inv);
		_manual_sp.obsavoid_switch = get_rc_sw3pos_position(rc_channels_s::RC_CHANNELS_FUNCTION_OBSAVOIDSWITCH,
					     _parameters.rc_obsavoid_th, _parameters.rc_obsavoid_inv,
					     _parameters.rc_obsavoid_mid_th, _parameters.rc_obsavoid_mid_inv);

		// publish actua
		publish_manual_inputs();

		/* Update parameters from RC Channels (tuning with RC) if activated */
		if (hrt_elapsed_time(&_last_rc_to_param_map_time) > 1e6) {
			set_params_from_rc(parameter_handles);
			_last_rc_to_param_map_time = hrt_absolute_time();
		}
	}
}

void
RCUpdate::publish_manual_inputs()
{
	int instance;

	/* publish manual_control_setpoint topic */
	orb_publish_auto(ORB_ID(manual_control_setpoint), &_manual_control_pub, &_manual_sp, &instance,
			 ORB_PRIO_HIGH);

	/* copy from mapped manual control to control group 3 */
	struct actuator_controls_s actuator_group_3 = {};

	actuator_group_3.timestamp = _manual_sp.timestamp;
	actuator_group_3.control[0] = _manual_sp.y;
	actuator_group_3.control[1] = _manual_sp.x;
	actuator_group_3.control[2] = _manual_sp.r;
	actuator_group_3.control[3] = _manual_sp.z;
	actuator_group_3.control[4] = _manual_sp.flaps;
	actuator_group_3.control[5] = _manual_sp.aux1;
	actuator_group_3.control[6] = _manual_sp.aux2;
	actuator_group_3.control[7] = _manual_sp.aux3;

	/* publish actuator_controls_3 topic */
	orb_publish_auto(ORB_ID(actuator_controls_3), &_actuator_group_3_pub, &actuator_group_3, &instance,
			 ORB_PRIO_DEFAULT);
}

void
RCUpdate::publish_rc_channels()
{
	int instance;
	orb_publish_auto(ORB_ID(rc_channels), &_rc_pub, &_rc, &instance, ORB_PRIO_DEFAULT);
}