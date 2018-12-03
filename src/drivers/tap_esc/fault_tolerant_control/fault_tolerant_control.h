/*
 * fault_tolerant_control.h
 *
 *  Created on: Mar 12, 2017
 *      Author: Haohua Chang
 */

#ifndef FAULT_TOLERANT_CONTROL_H_
#define FAULT_TOLERANT_CONTROL_H_

#include <stdint.h>
#include <parameters/param.h>

#define ESC_SUPPORT_REVERSE

class FaultTolerantControl
{
public:

	/**
	 * Constructor
	 */
	FaultTolerantControl();

	/**
	 * Destructor
	 */
	~FaultTolerantControl();

	/**
	 * the proportion according to S-Curve calculate compensatory gain
	 */
	float s_curve_gain(float x);

	/**
	 * Get parameter handle.
	 */
	int		get_parameters_handle();

	/**
	 * Update our local parameter cache.
	 */
	int		get_parameters_update();

	/**
	 * Check for parameter update and handle it.
	 */
	void 	parameter_update_poll();

	/**
	 * recalculate pwm output,when the vehicle enter fault tolerant control
	 */
	uint16_t recalculate_pwm_outputs(uint16_t failure_pwm, uint16_t diagonal_pwm, float &delta);
private:

	struct {
		param_t motor_failure_gain;
		param_t propeller_reverse_coefficient;
	}	_params_handles;		/**< handles for interesting parameters */

	struct {
		float motor_failure_gain;
		float propeller_reverse_coefficient;
	}	_params;

	int _params_sub;			/**< parameter updates subscription */
};


#endif /* FAULT_TOLERANT_CONTROL_H_ */
