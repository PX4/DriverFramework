/****************************************************************************
 *
 *   Copyright (C) 2017 PX4 Development Team. All rights reserved.
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

/*
 * I2C and GPIO interface for the MT9V117 image sensor used on Parrot Bebop 2
 */

#pragma once

#include "PWM.hpp"
#include "GPIO.hpp"

#include <stdint.h>
#include "I2CDevObj.hpp"

#define IMAGE_DEVICE_PATH "/dev/i2c-mt9v117"
#define IMAGE_CLASS_PATH "/dev/img"

#define DRV_DF_DEVTYPE_MT9V117 0x91
#define MT9V117_SLAVE_ADDRESS 0x5D

// The interface is only used to set parameter. A periodic callback is not needed but an interval of
// zero is not possible
#define MT9V117_MEASURE_INTERVAL_US 100000000

namespace DriverFramework
{

class MT9V117 : public I2CDevObj
{
public:
	MT9V117(const char *device_path)
		: I2CDevObj("ImageSensor", device_path, IMAGE_CLASS_PATH, MT9V117_MEASURE_INTERVAL_US),
		  _sensor_clock(9), _sensor_reset(129)
	{}
	~MT9V117() = default;

	int start();

	int stop();

protected:
	void _measure();
	void _publish();

private:
	int mt9v117_init();
	int probe();
	int soft_reset();
	int write_patch();
	int write_settings();
	int check_config_change(uint16_t new_state);
	int configure_sensor();
	int set_format();

	int write8(uint16_t add, uint8_t val);
	int write16(uint16_t add, uint16_t val);
	int write32(uint16_t add, uint32_t val);
	int read16(uint16_t add, uint16_t *val);

	// Helper functions
	uint16_t to_reg(uint16_t address, uint16_t offset) {return (0x8000 | (address << 10) | offset);}
	static uint32_t swap32(uint32_t val) { return (val >> 24) | ((val >> 8) & 0x0000FF00) | ((val << 8) & 0x00FF0000) | (val << 24); }

	PWM _sensor_clock;
	GPIO _sensor_reset;

};

}
