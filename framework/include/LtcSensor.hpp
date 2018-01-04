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

/***************************************************
 * modified by Christoph Tobler <christoph@px4.io>
 ***************************************************/

#pragma once

#include <stdint.h>
#include "SyncObj.hpp"
#include "I2CDevObj.hpp"

#define LTC2946_CLASS_PATH  "/dev/ltc"

#define LTC2946_MEASURE_INTERVAL_US 20000  // 1000000/50Hz

namespace DriverFramework
{

/** * The sensor independent data structure containing LTC2946 values. */
struct ltc2946_sensor_data {
	// 5V sensor
	float board_voltage_V; // [V]
	float board_current_A; // [A]
	// Battery sensor
	float battery_voltage_V; // [V]
	float battery_current_A; // [A]

	float remain;
	uint64_t read_counter; /*! the total number of sensor readings since the system was started */
	uint64_t error_counter;
};

class LtcSensor : public I2CDevObj
{
public:
	LtcSensor(const char *device_path, unsigned int sample_interval_usec) :
		I2CDevObj("LtcSensor", device_path, LTC2946_CLASS_PATH, LTC2946_MEASURE_INTERVAL_US)
	{}

	~LtcSensor() {}

	static int getSensorData(DevHandle &h, struct ltc2946_sensor_data &out_data, bool is_new_data_required)
	{
		LtcSensor *me = DevMgr::getDevObjByHandle<LtcSensor>(h);
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

	struct ltc2946_sensor_data		m_sensor_data;
	SyncObj 			m_synchronize;
};

}; // namespace DriverFramework
