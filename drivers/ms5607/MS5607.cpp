/****************************************************************************
 *
 *   Copyright (C) 2015 Mark Charlebois. All rights reserved.
 *   Copyright (C) 2016 PX4 Development Team. All rights reserved.
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

#include <string.h>
#include "DriverFramework.hpp"
#include "MS5607.hpp"
#ifdef __QURT
#include "dev_fs_lib_i2c.h"
#endif

#define MS5607_REG_ID 0xD0
#define MS5607_REG_CTRL_MEAS 0xF4
#define MS5607_REG_CONFIG 0xF5
#define MS5607_REG_PRESS_MSB 0xF7

#define MS5607_ID 0x58

#define MS5607_BITS_CTRL_MEAS_OVERSAMPLING_TEMP2X     0b01000000
#define MS5607_BITS_CTRL_MEAS_OVERSAMPLING_PRESSURE8X 0b00010000
#define MS5607_BITS_CTRL_MEAS_POWER_MODE_NORMAL	      0b00000011

#define MS5607_BITS_CONFIG_STANDBY_0MS5	0b00000000
#define MS5607_BITS_CONFIG_FILTER_OFF	0b00000000
#define MS5607_BITS_CONFIG_SPI_OFF	0b00000000

using namespace DriverFramework;

// convertPressure must be called after convertTemperature
// as convertTemperature sets m_sensor_data.t_fine
int64_t MS5607::convertPressure(int64_t adc_P)
{
	return -1;
}

int32_t MS5607::convertTemperature(int32_t adc_T)
{
	return -1;
}

int MS5607::loadCalibration()
{
	return -1;
}

int MS5607::ms5607_init()
{
  return -1;
}

int MS5607::start()
{
	return -1;
}

int MS5607::stop()
{
	return -1;
}

void MS5607::_measure(void)
{
}

int MS5607::_publish(struct baro_sensor_data &data)
{
	// TBD
	return -1;
}
