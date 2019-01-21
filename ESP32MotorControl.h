/* Header for ESP32MotorControl
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
 * This header file describes the public API for SerialDebug.
 *
 */

#ifndef ESP32MotorControl_H
#define ESP32MotorControl_H

#include "Arduino.h"

//////// Defines

#ifndef ESP32
	#error "this library is only for ESP32"
#endif

#define PWM_FREQ 1000	// PWM Frequency		

//////// Class

class ESP32MotorControl
{
public:

	// Fields

	uint16_t mMotorSpeed[2] = {0, 0};
	boolean mMotorForward[2] = {true, true};

	// Methods:

	void attachMotor(uint8_t gpioIn1, uint8_t gpioIn2);
	void attachMotors(uint8_t gpioIn1, uint8_t gpioIn2, uint8_t gpioIn3, uint8_t gpioIn4);

	void motorFullForward(uint8_t motor);
	void motorForward(uint8_t motor, uint8_t speed);
	void motorFullReverse(uint8_t motor);
	void motorReverse(uint8_t motor, uint8_t speed);
	void motorStop(uint8_t motor);

	void motorsStop();

	void handle();

	uint8_t getMotorSpeed(uint8_t motor);
	boolean isMotorForward(uint8_t motor);
	boolean isMotorStopped(uint8_t motor);

private:

	// Fields:

	boolean mMotorAttached[2] = {false, false};

	// Methods

	boolean isMotorValid(uint8_t motor);
};

#endif // ESP32MotorControl_H
