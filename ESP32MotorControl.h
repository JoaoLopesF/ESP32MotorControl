/* Header for ESP32MotorControl
 *
 * Copyright (C) 2021  Karol Pieniacy https://github.com/pieniacy/ESP32MotorControl
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

#ifndef ESP32
#error "this library is only for ESP32"
#endif

class ESP32MotorControl {
public:
	// Fields:
	uint16_t mMotorSpeed[2] = {0, 0};
	boolean mMotorForward[2] = {true, true};

	// Methods:

	// Attach one motor
	void attachMotor(uint8_t gpioIn1, uint8_t gpioIn2, uint32_t frequencyHz = 1000);
	// Attach two motors
	void attachMotors(uint8_t gpioIn1, uint8_t gpioIn2, uint8_t gpioIn3, uint8_t gpioIn4,
	                  uint32_t frequencyHz = 1000);

	// Set speed -> PWM duty in the range 0-100
	void motorForward(uint8_t motor, uint8_t speed);
	void motorReverse(uint8_t motor, uint8_t speed);
	// Set full speed
	void motorFullForward(uint8_t motor);
	void motorFullReverse(uint8_t motor);
	// Stop specific motor
	void motorStop(uint8_t motor);
	// Stop both motors
	void motorsStop();
	// Set speed values of [-100 : 100] to both motors
	void motorsSet(int8_t speed0, int8_t speed1);

	uint8_t getMotorSpeed(uint8_t motor);
	boolean isMotorForward(uint8_t motor);
	boolean isMotorStopped(uint8_t motor);

private:
	// Fields:
	boolean mMotorAttached_[2] = {false, false};

	// Methods:
	void setMotor_(uint8_t motor, int8_t speed);
	boolean isMotorValid_(uint8_t motor);
};

#endif  // ESP32MotorControl_H
