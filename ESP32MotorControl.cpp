
/*****************************************
 * Library   : ESP32MotorControl - Library for dual motor driver carrier for Arduino.
 * Programmer: Joao Lopes
 * Comments  : This library is to use with dual motors ICs, as DRV8833, DRV8825 and L298.
 * 			   And uses the MCPWM for control motors
 * Versions  :
 * ------ 	---------- 		-------------------------
 * 0.1.0  	2018-12-26		First beta
 *****************************************/

/*
 * Source for ESP32MotorControl
 *
 * Copyright (C) 2018  Joao Lopes https://github.com/JoaoLopesF/ESP32MotorControl
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * This file contains the code for ESP32MotorControl library.
 *
 */

/*
 * TODO list:
 * - Port to L298
 */

/*
 * TODO known issues:
 */

////// Includes

#include "Arduino.h"

#include "ESP32MotorControl.h"

#include <stdio.h>

#include "esp_system.h"
#include "esp_attr.h"

#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

///// Defines

// debug

//#define debug(fmt, ...)
#define debug(fmt, ...) Serial.printf("%s: " fmt "\r\n", __func__, ##__VA_ARGS__)

///// Methods

///// Driver with 4 pins: DRV88nn, DRV8833, DRV8825, etc.

// Attach one motor

void ESP32MotorControl::attachMotor(uint8_t gpioIn1, uint8_t gpioIn2)
{
	attachMotors(gpioIn1, gpioIn2, 0, 0);
}

// Attach two motors

void ESP32MotorControl::attachMotors(uint8_t gpioIn1, uint8_t gpioIn2, uint8_t gpioIn3, uint8_t gpioIn4)
{
	// debug

	debug("init MCPWM Motor 0");

	// Attach motor 0 input pins.

	// Set MCPWM unit 0

    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, gpioIn1);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, gpioIn2);

	// Indicate the motor 0 is attached.

	this->mMotorAttached[0] = true;

	// Attach motor 1 input pins.

	if (!(gpioIn3 == 0 && gpioIn4 ==0)) {

		debug("init MCPWM Motor 1");

		// Set MCPWM unit 1
    
		mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1A, gpioIn3);
    	mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1B, gpioIn4);

		// Indicate the motor 1 is attached.

		this->mMotorAttached[1] = true;
	}

    // Initial MCPWM configuration

    debug ("Configuring Initial Parameters of MCPWM...");

    mcpwm_config_t pwm_config;
    pwm_config.frequency = PWM_FREQ;    //frequency,
    pwm_config.cmpr_a = 0;    		//duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    		//duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings

    mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_1, &pwm_config);    //Configure PWM1A & PWM1B with above settings

	debug ("MCPWM initialized");
}

// Motor full forward

void ESP32MotorControl::motorFullForward(uint8_t motor)
{
	if (!isMotorValid(motor)) {
		return;
	}

	// Full forward

	if (motor == 0) {

		mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B);
		mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);

	} else {

		mcpwm_set_signal_low(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_B);
		mcpwm_set_signal_high(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A);
	}

	mMotorSpeed[motor] = 100; // Save it
	mMotorForward[motor] = true;

	debug ("Motor %u full forward", motor);
}

// Motor set speed forward

void ESP32MotorControl::motorForward(uint8_t motor, float speed)
{
	if (!isMotorValid(motor)) {
		return;
	}

	if (speed == 100) { // Full speed

		motorFullForward(motor);

	} else {

		// Set speed -> PWM duty 0-100

		if (motor == 0) {

			mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B);
			mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, speed);
			mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state

		} else {

			mcpwm_set_signal_low(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_B);
			mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, speed);
			mcpwm_set_duty_type(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state

		}

		mMotorSpeed[motor] = speed; // Save it
		mMotorForward[motor] = true;

		debug("Motor %u forward speed %u", motor, speed);
	}
}

void ESP32MotorControl::motorFullReverse(uint8_t motor)
{
	if (!isMotorValid(motor)) {
		return;
	}

	// Full forward

	if (motor == 0) {

		mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
		mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B);

	} else {

		mcpwm_set_signal_low(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A);
		mcpwm_set_signal_high(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_B);
	}

	mMotorSpeed[motor] = 100; // Save it
	mMotorForward[motor] = false;

	debug ("Motor %u full reverse", motor);
}

// Motor set speed reverse

void ESP32MotorControl::motorReverse(uint8_t motor, uint8_t speed)
{
	if (!isMotorValid(motor)) {
		return;
	}

	if (speed == 100) { // Full speed

		motorFullReverse(motor);

	} else {


		// Set speed -> PWM duty 0-100

		if (motor == 0) {

			mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
			mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, speed);
			mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state

		} else {

			mcpwm_set_signal_low(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A);
			mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_B, speed);
			mcpwm_set_duty_type(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_B, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state

		}

		mMotorSpeed[motor] = speed; // Save it
		mMotorForward[motor] = false;

		debug("Motor %u reverse speed %u", motor, speed);
	}
}


// Motor stop

void ESP32MotorControl::motorStop(uint8_t motor)
{
	if (!isMotorValid(motor)) {
		return;
	}

	// Motor stop

	if (motor == 0) {

		mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
		mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B);

	} else {

		mcpwm_set_signal_low(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A);
		mcpwm_set_signal_low(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_B);

	}

	mMotorSpeed[motor] = 0; // Save it
	mMotorForward[motor] = true; // For stop

	debug("Motor %u stop", motor);
}

// Motors stop

void ESP32MotorControl::motorsStop()
{
	motorStop(0);
	motorStop(1);

	debug("Motors stop");
}

// Get motor speed

uint8_t ESP32MotorControl::getMotorSpeed(uint8_t motor) {

	if (!isMotorValid(motor)) {
		return false;
	}
	return mMotorSpeed[motor];

}

// Is motor in forward ?

boolean ESP32MotorControl::isMotorForward(uint8_t motor) {
	
	if (!isMotorValid(motor)) {
		return false;
	}

	if (isMotorStopped(motor)) {
		return false;
	} else {
		return mMotorForward[motor];
	}
}

// Is motor stopped ?

boolean ESP32MotorControl::isMotorStopped(uint8_t motor) {
	

	if (!isMotorValid(motor)) {
		return true;
	}
	return (mMotorSpeed[motor] == 0);
}

//// Privates

// Is motor valid ?

boolean ESP32MotorControl::isMotorValid(uint8_t motor) {


	if (motor > 1) {
		return false;
	}

	return mMotorAttached[motor];
}


///// End
