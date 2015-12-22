/****************************************************************************
 *
 *   Copyright (C) 2015 Mark Charlebois. All rights reserved.
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

#pragma once

#include <stdint.h>
#include "SyncObj.hpp"
#include "I2CDevObj.hpp"
#include "dev_fs_lib_i2c.h"

#define PRESSURE_DEVICE_PATH "/dev/i2c-2"
#define PRESSURE_CLASS_PATH  "/dev/pressure"

namespace DriverFramework {

/**
 * The sensor independent data structure containing pressure values.
 */
struct pressure_sensor_data
{
	int32_t  t_fine; 			/*! used internally to calculate a temperature compensated pressure value. */
	float    pressure_in_pa; 		/*! current pressure in Pascals */
	float    temperature_in_c; 		/*! current temperature in C at which the pressure was read */
	uint32_t sensor_read_counter;		/*! the total number of pressure sensor readings since the system was started */
	uint64_t last_read_time_in_usecs; 	/*! time stamp indicating the time at which the pressure in this data structure was read */
	uint64_t error_count; 			/*! the total number of errors detected when reading the pressure, since the system was started */
};

class PressureSensor : public I2CDevObj
{
public:
	PressureSensor(const char *device_path, unsigned int sample_interval_usec) :
		I2CDevObj("PressureSensor", device_path, PRESSURE_CLASS_PATH, sample_interval_usec)
	{}

	~PressureSensor() {}

	void setAltimeter(float altimeter_setting_in_mbars)
	{
		m_altimeter_mbars = altimeter_setting_in_mbars;
	}

	static int getSensorData(DevHandle &h, struct pressure_sensor_data &out_data, bool is_new_data_required)
	{
		PressureSensor *me = DevMgr::getDevObjByHandle<PressureSensor>(h);
		int ret = -1;
		if (me != nullptr) {
			me->m_synchronize.lock();
			if (is_new_data_required) {
				me->m_synchronize.waitOnSignal(0);
			}
			out_data = me->m_sensor_data;
			me->m_synchronize.unlock();
			ret = 0;
		}

		return ret;
	}

protected:
	virtual void _measure() = 0;

	struct dspal_i2c_ioctl_slave_config 	m_slave_config;
	struct pressure_sensor_data 		m_sensor_data;
	float 					m_altimeter_mbars = 0.0;
	SyncObj 				m_synchronize;
};

}; // namespace DriverFramework
