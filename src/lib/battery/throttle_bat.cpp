#include "throttle_bat.hpp"
#include <mathlib/mathlib.h>
// Yuneec-specific
#include <systemlib/mavlink_log.h>

void
BatteryThrottle::update(hrt_abstime timestamp,
			float voltage_v,
			float current_a,
			bool connected,
			bool selected_source,
			int priority,
			float throttle_normalized,
			bool armed,
			battery_status_s *battery_status)
{

	reset(battery_status);
	battery_status->timestamp = timestamp;

	if (!_battery_initialized) {
		filter1order(_voltage_filtered_v, voltage_v, 0.01f);
		filter1order(_current_filtered_a, current_a, 0.02f);
		filter1order(_throttle_filtered, throttle_normalized, 0.01f);

	}

	sumDischarged(timestamp, current_a);
	estimateRemaining(_voltage_filtered_v, _current_filtered_a, _throttle_filtered, armed);
	computeScale();
	computeRemainingTime(current_a);

	if (_battery_initialized) {
		determineWarning(_warning, connected, _remaining);
	}

	bool reliably_connected = true;

#ifdef BOARD_HAS_POWER_CHECK

	if (_link_check.get()) {
		reliably_connected = !stm32_gpioread(POWER_CHECK_GPIO);
	}

#endif

	if (_voltage_filtered_v > 2.1f) {
		_battery_initialized = true;
		battery_status->voltage_v = voltage_v;
		battery_status->voltage_filtered_v = _voltage_filtered_v;
		battery_status->scale = _scale;
		battery_status->time_remaining_s = _time_remaining_s;
		battery_status->current_a = current_a;
		battery_status->current_filtered_a = _current_filtered_a;
		battery_status->discharged_mah = _discharged_mah;
		battery_status->warning = _warning;
		battery_status->remaining = _remaining;
		battery_status->connected = connected;
		battery_status->system_source = selected_source;
		battery_status->priority = priority;
		battery_status->reliably_connected = reliably_connected;
		battery_status->tethered = _tethered;
	}

	// Yuneec-specific [ch4207]: Consider voltage above usual battery voltrage
	// as an infinite power supply. Can be used for tethering for example.
	if (_v_tether.get() >= 0.0f && _voltage_filtered_v >= _v_tether.get()) {
		if (!_tethered) {
			_tethered = true;
			mavlink_log_info(&_mavlink_log_pub, "Tethered power supply: enabled");
		}

		// Override battery estimate while thethered
		battery_status->time_remaining_s = -1.f;
		battery_status->warning = battery_status_s::BATTERY_WARNING_NONE;
		battery_status->remaining = 1.f;
	}

	// For now we don't support switching from thethering to a backup battery
	// else if (_tethered) {
	// 	_tethered  = false;
	// 	_discharged_mah = 0;  // reset internal state when tether disconnects
	// 	mavlink_log_info(&_mavlink_log_pub, "Tethered power supply: disabled");
	// }
}

void
BatteryThrottle::sumDischarged(hrt_abstime timestamp, float current_a)
{
	// Not a valid measurement
	if (current_a < 0.f) {
		// Because the measurement was invalid we need to stop integration
		// and re-initialize with the next valid measurement
		_last_timestamp = 0;
		return;
	}

	// Ignore first update because we don't know dt.
	if (_last_timestamp != 0) {
		const float dt = (timestamp - _last_timestamp) / 1e6;
		// mAh since last loop: (current[A] * 1000 = [mA]) * (dt[s] / 3600 = [h])
		_discharged_mah_loop = (current_a * 1e3f) * (dt / 3600.f);
		_discharged_mah += _discharged_mah_loop;
	}

	_last_timestamp = timestamp;
}

void
BatteryThrottle::estimateRemaining(float voltage_v, float current_a, float throttle, bool armed)
{
	// remaining battery capacity based on voltage
	float cell_voltage = voltage_v / _n_cells.get();

	// correct battery voltage locally for load drop to avoid estimation fluctuations
	if (_r_internal.get() >= 0.f) {
		cell_voltage += _r_internal.get() * current_a;

	} else {
		// assume linear relation between throttle and voltage drop
		cell_voltage += throttle * _v_load_drop.get();
	}

	_remaining_voltage = math::gradual(cell_voltage, _v_empty.get(), _v_charged.get(), 0.f, 1.f);

	// choose which quantity we're using for final reporting
	if (_capacity.get() > 0.f) {
		// if battery capacity is known, fuse voltage measurement with used capacity
		if (!_battery_initialized) {
			// initialization of the estimation state
			_remaining = _remaining_voltage;

		} else {
			// The lower the voltage the more adjust the estimate with it to avoid deep discharge
			const float weight_v = 3e-4f * (1 - _remaining_voltage);
			_remaining = (1 - weight_v) * _remaining + weight_v * _remaining_voltage;
			// directly apply current capacity slope calculated using current
			_remaining -= _discharged_mah_loop / _capacity.get();
			_remaining = math::max(_remaining, 0.f);
		}

	} else {
		// else use voltage
		_remaining = _remaining_voltage;
	}
}

void
BatteryThrottle::computeScale()
{
	const float voltage_range = (_v_charged.get() - _v_empty.get());

	// reusing capacity calculation to get single cell voltage before drop
	const float bat_v = _v_empty.get() + (voltage_range * _remaining_voltage);

	_scale = _v_charged.get() / bat_v;

	if (_scale > 1.3f) { // Allow at most 30% compensation
		_scale = 1.3f;

	} else if (!PX4_ISFINITE(_scale) || _scale < 1.f) { // Shouldn't ever be more than the power at full battery
		_scale = 1.f;
	}
}

void
BatteryThrottle::computeRemainingTime(float current_a)
{
	/* Only estimate remaining time with useful in flight current measurements */
	if (_current_filtered_a > 1.f) {
		/* Initialize strongly filtered current to an estimated average consumption */
		if (_current_filtered_a_for_time < 0.f) {
			/* TODO: better initial value based on "hover current" from last flight */
			_current_filtered_a_for_time = 15.f;
		}

		/* Filter current very strong, we basically want the average consumption */
		const float weight_ct = 5e-5f;
		_current_filtered_a_for_time = (1 - weight_ct) * _current_filtered_a_for_time + weight_ct * current_a;

		/* Remaining time estimation only possible with capacity */
		if (_capacity.get() > 0.f) {
			const float remaining_capacity_mah = _remaining * _capacity.get();
			const float current_ma = _current_filtered_a_for_time * 1e3f;
			_time_remaining_s = remaining_capacity_mah / current_ma * 3600.f;
		}

	} else {
		_current_filtered_a_for_time = -1.f;
		_time_remaining_s = -1.f;
	}
}
