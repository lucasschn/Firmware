/****************************************************************************
 *
 *   Copyright (c) 2013-2017 PX4 Development Team. All rights reserved.
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
 * @file five_propellers_handling_params.c
 * Parameters for multicopter breakdown(five propellers mode)attitude controller
 *
 * @author Haohua Chang
 */

/**
 * FTC configuration enable
 *
 * Flag for enabling fault-tolerant-control. This has no effect on boards that
 * do not support FTC.
 *
 * @value 0 Disabled
 * @value 1 Enabled
 *
 * @group Fault Tolerant Control
 * @reboot_required true
 */
PARAM_DEFINE_INT32(FTC_ENABLE, 1);

/**
 * motor failure P gain compensate for fault tolerant control
 *
 * fault tolerant control handling when the motor(e.g motor0) have failure,error_pwm = (motor1 pwm - motor0 pwm)*(proportional gain compensate)
 *
 * @min 0.0
 * @max 1.0
 * @decimal 3
 * @increment 0.01
 * @group Fault Tolerant Control
 * @reboot_required true
 */
PARAM_DEFINE_FLOAT(FTC_GAIN, 0.6f);

/**
 * the develop force effects when corotate propeller is equivalent to
 * the develop force effects when reverse propeller multiply by compensate coefficient.
 *
 * corotate propeller force = (reverse propellers force)*(proportional reverse compensatory coefficient)
 *
 * @min 0.0
 * @max 5.0
 * @decimal 3
 * @increment 0.01
 * @group Fault Tolerant Control
 * @reboot_required true
 */
PARAM_DEFINE_FLOAT(PROPELL_REV_COEF, 1.2f);
