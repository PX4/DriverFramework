/****************************************************************************
 *
 *   Copyright (C) 2016 Julian Oes. All rights reserved.
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

#include "MagSensor.hpp"

namespace DriverFramework
{

#define MAG_DEVICE_PATH "/dev/iic-2"

// 150 Hz (supported in single measurment mode is up to 160 Hz
#define HMC5883_MEASURE_INTERVAL_US (1000000/150)

// TODO: include some common header file (currently in drv_sensor.h).
#define DRV_DF_DEVTYPE_HMC5883 0x43

#define HMC5883_SLAVE_ADDRESS		(0x1e)


class HMC5883 : public MagSensor
{
public:
	HMC5883(const char *device_path) :
		MagSensor(device_path, HMC5883_MEASURE_INTERVAL_US),
		_measurement_requested(false)
	{
		m_id.dev_id_s.devtype = DRV_DF_DEVTYPE_HMC5883;
		m_id.dev_id_s.address = HMC5883_SLAVE_ADDRESS;
	}

	// @return 0 on success, -errno on failure
	virtual int start();

	// @return 0 on success, -errno on failure
	virtual int stop();

protected:
	virtual void _measure();
	virtual int _publish(struct mag_sensor_data &data);

private:
	int loadCalibration();

	// returns 0 on success, -errno on failure
	int hmc5883_init();

	//struct hmc5883_sensor_calibration 	m_sensor_calibration;

	// we need to request a measurement before we can collect it
	bool _measurement_requested;
};

}; // namespace DriverFramework
