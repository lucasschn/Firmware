#include "ekf_bat.hpp"
#include <lib/mathlib/mathlib.h>

using namespace matrix;

void
BatteryEKF::updateStatus(hrt_abstime timestamp, float voltage_v, float current_a,
				bool connected, bool selected_source, int priority,
				float throttle_normalized,
				bool armed, battery_status_s *battery_status)
{
	if (_init_kf) {
		this->kfInit(voltage_v);
		_init_kf = false; // mark the Kalman filter initialization as done
	}

	if (_battery_initialized) {
		filter1order(_voltage_filtered_v, voltage_v, 0.01f);
		filter1order(_current_filtered_a, current_a, 0.02f);

	} else {
		_voltage_filtered_v = voltage_v;
		_current_filtered_a = current_a;
	}

	kfUpdate(timestamp, _current_filtered_a, voltage_v);

	// To be uncommented when the estimator will be reliable
	// if (_battery_initialized) {
	// 	determineWarning(_warning, connected, _remaining_ekf);
	// }

	// for logging purpose
	if (_voltage_filtered_v > 2.1f) {
		_battery_initialized = true;
	}
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
}

void
BatteryEKF::kfInit(float voltage_v)
{
	_yhat = voltage_v / _n_cells.get();
	// Battery is considered at equilibrium when booting up (if last value from last time could be saved would be nice). That means the sensed voltage is assumed to be equal to the Open Circuit Voltage (OCV).
	_SOC0 = this->soc_from_ocv(voltage_v / _n_cells.get());
	_iR10 = 0.0f;
	// Matrices initialization
	_batmat_A(0, 0) = 1.0f;
	_batmat_A(0, 1) = 0.0f;
	_batmat_A(1, 0) = 0.0f;
	_batmat_A(1, 1) = exp(-_deltatime / (_R1.get() * _C1.get()));
	_batmat_C(0, 0) = this->getSlope(_SOC0);
	_batmat_C(0, 1) = -_R1.get();
	_xhat(0) = _SOC0;
	_xhat(1) = _iR10;
	_covx(0, 0) = 0.1f;
	_covx(0, 1) = 0.f;
	_covx(1, 0) = 0.f;
	_covx(1, 1) = 0.f;
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
	_deltatime = (timestamp - _last_timestamp) / 1e6;
	this->recompute_statespace(_deltatime);
	SquareMatrix<float, 1> tmp;

	// 1c Computation of future yhat using future state and last input would not make sense !
	_yhat = ocv_from_soc(_xhat(0)) - _R1.get() * _xhat(1) - _R0.get() * _u;

	// 1b
	_covx = _batmat_A * _covx * _batmat_A.T() + _covw; // (numerical robustness) no risk of losing symetry here

	// 2a
	covxy = _covx * _batmat_C.T();
	tmp = _batmat_C * _covx * _batmat_C.T();
	_covy = tmp(0, 0) + _covv; // scalar, so not computationally demanding
	_kalman_gain = covxy / _covy;


	_innovation = _y - _yhat;

	// Innovation saturation
	if (_innovation > 0.05f) {
		_innovation = 0.05f;

	} else if (_innovation < -0.05f) {
		_innovation = -0.05f;
	}

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
BatteryEKF::ocv_from_soc(float SOC)
{
	float OCV = 0;

	for (unsigned int poly_expo = 0; poly_expo <= sizeof(_poly_coeff) - 1; poly_expo++) {
		OCV += _poly_coeff[sizeof(_poly_coeff) - 1 - poly_expo] * powf(SOC, (float) poly_expo);
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
	float const dOCV = this->ocv_from_soc(min(1.0f, SOC + 0.01f)) - this->ocv_from_soc(max(0.0f, SOC - 0.01f));
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

