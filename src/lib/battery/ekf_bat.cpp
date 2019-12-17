#include "ekf_bat.hpp"
#include <lib/mathlib/mathlib.h>

using namespace matrix;



void
BatteryEKF::updateStatus(hrt_abstime timestamp, float voltage_v, float current_a,
			 bool connected, bool selected_source, int priority,
			 float throttle_normalized,
			 bool armed, battery_status_s *battery_status)
{

	// Check wheter the estimator has all required information
	if (!connected || !inputsValid(timestamp, voltage_v, current_a)) {
		reset(battery_status);
		_last_timestamp = timestamp;
		return;
	}

	// update deltatime
	_deltatime = (timestamp - _last_timestamp) / 1e6;

	// Initialize filter states
	if (!_initialized) {
		_voltage_filtered_v = voltage_v;
		_current_filtered_a = current_a;
		kfInit(voltage_v, current_a);
		_initialized = true;
	}

	// Everything is ready. Update filters.
	filter1order(_voltage_filtered_v, voltage_v, 0.01f);
	filter1order(_current_filtered_a, current_a, 0.02f);

	// update kalman
	kfUpdate(timestamp, _current_filtered_a, voltage_v);

	// update warning
	determineWarning(_warning, _xhat(0));

	// check if battery has "reliably-connected-feature"
	bool reliably_connected = true;

#ifdef BOARD_HAS_POWER_CHECK

	if (_link_check.get()) {
		reliably_connected = !stm32_gpioread(POWER_CHECK_GPIO);
	}

#endif

	// set battery_status_s
	// general
	battery_status->voltage_v = voltage_v;
	battery_status->voltage_filtered_v = _voltage_filtered_v;
	battery_status->scale = NAN; // unused
	battery_status->time_remaining_s = NAN; //unused
	battery_status->current_a = current_a;
	battery_status->current_filtered_a = _current_filtered_a;
	battery_status->warning = _warning;
	battery_status->discharged_mah = NAN; //unused
	battery_status->connected = connected;
	battery_status->system_source = selected_source;
	battery_status->priority = priority;
	battery_status->tethered = false;
	battery_status->reliably_connected = reliably_connected;
	// extended Kalman filter 
	battery_status->voltage_estimate = _yhat;
	battery_status->covx[0] = _covx(0, 0);
	battery_status->covx[1] = _covx(0, 1);
	battery_status->covx[2] = _covx(1, 0);
	battery_status->covx[3] = _covx(1, 1);
	battery_status->covw[0] = _covw(0, 0);
	battery_status->covw[1] = _covw(0, 1);
	battery_status->covw[2] = _covw(1, 0);
	battery_status->covw[3] = _covw(1, 1);
	battery_status->kalman_gain[0] = _kalman_gain(0, 0);
	battery_status->kalman_gain[1] = _kalman_gain(1, 0);
	battery_status->innovation = _innovation;
	battery_status->unsaturated_innovation = _y - _yhat;
	battery_status->remaining = _xhat(0);
	battery_status->resistor_current = _xhat(1);

	// update last timestamp
	_last_timestamp = timestamp;
}

void
BatteryEKF::kfInit(const float voltage_v, const float current_a)
{
	_yhat = voltage_v / _n_cells.get();
	// Battery is considered at equilibrium when booting up (if last value from last time could be saved would be nice). That means the sensed voltage is assumed to be equal to the Open Circuit Voltage (OCV).
	float SOC0 = soc_from_ocv(voltage_v / _n_cells.get() + (_R0.get() + _R1.get()) * current_a);
	float iR10 = 0.0f; // battery is considered at equilibrium at bootup
	// Matrices initialization
	_batmat_A(0, 0) = 1.0f;
	_batmat_A(0, 1) = 0.0f;
	_batmat_A(1, 0) = 0.0f;
	_batmat_A(1, 1) = exp(-_deltatime / (_R1.get() * _C1.get()));
	_batmat_C(0, 0) = getSlope(SOC0);
	_batmat_C(0, 1) = -_R1.get();
	_xhat(0) = SOC0;
	_xhat(1) = iR10;
	_covx(0, 0) = 0.1f;
	_covx(0, 1) = 0.f;
	_covx(1, 0) = 0.f;
	_covx(1, 1) = 0.01f;
	_covw(0, 0) = 1e-3f;
	_covw(0, 1) = 0.0f;
	_covw(1, 0) = 0.0f;
	_covw(1, 1) = 1e-4f;
	PX4_INFO("Kalman filter has been initialized.");
}

void
BatteryEKF::kfUpdate(hrt_abstime timestamp, float current_a, float voltage_v)
{	
	_u = current_a;
	_y = voltage_v / _n_cells.get(); // Voltage per cell is required
	recompute_statespace(_deltatime);
	SquareMatrix<float, 1> tmp;

	// 1c Computation of future yhat using future state and last input would not make sense !
	_yhat = ocv_from_soc(_xhat(0), _poly_coeff, 12u) - _R1.get() * _xhat(1) - _R0.get() * _u;

	// 1b
	_covx = _batmat_A * _covx * _batmat_A.T() + _covw; // (numerical robustness) no risk of losing symetry here

	// 2a
	covxy = _covx * _batmat_C.T();
	tmp = _batmat_C * _covx * _batmat_C.T();
	_covy = tmp(0, 0) + _covv; // scalar, so not computationally demanding
	_kalman_gain = covxy / _covy;


	_innovation = _y - _yhat;

	if (abs(_innovation>2.0f)){
		_error_count++;
	} else {
		_error_count = 0;
	}
	if (_error_count >= 200){
		_covx = _covx*5.0f;
		_error_count = 0;
	}

	// Innovation saturation
	//if (_innovation > 0.05f) {
	//	_innovation = 0.05f;
//
	//} else if (_innovation < -0.05f) {
	//	_innovation = -0.05f;
	//}

	// 1a & 2b Computation of xhat using last input
	_xhat = _batmat_A * _xhat + _batmat_B * _u + _kalman_gain * _innovation;

	/* Is it really the best way to constraint the state ?
	We could also project the unconstrainted state space
	onto a constrained one using a clever projection
	*/
	if (_xhat(0) > 1.0f) {
		_xhat(0) = 1.0f;

	} else if (_xhat(0) < 0.0f) {
		_xhat(0) = 0.0f;
	}

	// 2c (Joseph-form covariance update for improved numerical robustness)
	_covx = (I - _kalman_gain * _batmat_C) * _covx * (I - _kalman_gain * _batmat_C).T() + _kalman_gain * _covv *
		_kalman_gain.T(); // puts nonzero values in the antidiagonal of covx
	_last_timestamp = timestamp;
}


float
BatteryEKF::ocv_from_soc(const float SOC, const float *poly_coeff, unsigned int poly_length)
{
	float OCV = 0;

	for (unsigned int poly_expo = 0; poly_expo <= poly_length - 1; poly_expo++) {
		OCV += poly_coeff[poly_length - 1u - poly_expo] * powf(SOC, (float) poly_expo);
	}

	return OCV;
}

float
BatteryEKF::soc_from_ocv(float OCV)
{
	float SOC = -1;

	/* The OCV(SOC) curve is linearized by parts in three segments. Those values are stored in the _OCV_array and _SOC_array variables. The SOC is computed by linear interpolation, once the OCV has been identified to belong to one of these three segments through the if statements.
	*/
	if (OCV >= _OCV_array[0] and OCV <= _OCV_array[3]) {
		if (OCV >= _OCV_array[0] and OCV <= _OCV_array[1]) {
			SOC = (_SOC_array[1] - _SOC_array[0]) / (_OCV_array[1] - _OCV_array[0]) * (OCV - _OCV_array[0]);

		} else if (OCV > _OCV_array[1] and OCV <= _OCV_array[2]) {
			SOC = (_SOC_array[2] - _SOC_array[1]) / (_OCV_array[2] - _OCV_array[1]) * (OCV - _OCV_array[1]) + _SOC_array[1];

		} else if (OCV > _OCV_array[2] and OCV <= _OCV_array[3]) {
			SOC = (_SOC_array[3] - _SOC_array[2]) / (_OCV_array[3] - _OCV_array[2]) * (OCV - _OCV_array[2]) + _SOC_array[2];
		}
	}

	return SOC;
}

float
BatteryEKF::getSlope(float SOC)
{
	using math::min;
	using math::max;
	float const dSOC = min(1.0f, SOC + 0.01f) - max(0.0f, SOC - 0.01f);
	// min(1.0,z+0.01) is used to prevent z+0.01 exceeding 1 in case z>0.99. Having SOC>1 does not make any sense. Same on the other end of the curve.
	// +/-0.01 is an arbitrary value that give decent slope predictions in Jupyter notebook implementation.
	float const dOCV = ocv_from_soc(min(1.0f, SOC + 0.01f), _poly_coeff, 12u) - ocv_from_soc(max(0.0f, SOC - 0.01f), _poly_coeff, 12u);
	float slope = dOCV / dSOC;

	return slope;
}

void
BatteryEKF::recompute_statespace(float timedelta)
{
	_batmat_A(1, 1) = exp(-timedelta / (_R1.get() * _C1.get()));
	_batmat_B(0) = timedelta / (_capacity_mAh * 3.6f); // 3.6 converts from mAh to Coulombs
	_batmat_B(1) = 1 - exp(-timedelta / (_R1.get() * _C1.get()));
	_batmat_C(0, 0) = this->getSlope(_xhat(0));
}

bool BatteryEKF::inputsValid(hrt_abstime timestamp, float voltage_v, float current_a)
{
	// Ensure that we can compute deltatime
	if (!PX4_ISFINITE(_last_timestamp) || (_last_timestamp == 0)) {
		return false;
	}

	// Ensure that voltage is above 2.1 (TODO: add better criteria)
	if (voltage_v  < 2.1f) {
		return false;
	}

	// Ensure that current is positive
	if (current_a < 0.0f) {
		return false;
	}

	// Check if cell count is valid
	if (_n_cells.get() <= 0 || !PX4_ISFINITE(_n_cells.get())) {
		return false;
	}

	return true;
}

