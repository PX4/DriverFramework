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

#include "ImuSensor.hpp"


namespace DriverFramework
{

// TODO: use define from some include for this
#define	M_PI		3.14159265358979323846	/* pi */

// update frequency 500 Hz
#define MPU9250_MEASURE_INTERVAL_US 2000

// -2000 to 2000 degrees/s, 16 bit signed register, deg to rad conversion
#define GYRO_RAW_TO_RAD_S 	 (2000.0f / 32768.0f * M_PI / 180.0f)

// TODO: include some common header file (currently in drv_sensor.h).
#define DRV_DF_DEVTYPE_MPU9250 0x41

#define MPU_WHOAMI_9250			0x71


class MPU9250 : public ImuSensor
{
public:
	MPU9250(const char *device_path) :
		ImuSensor(device_path, MPU9250_MEASURE_INTERVAL_US)
	{
		m_id.dev_id_s.devtype = DRV_DF_DEVTYPE_MPU9250;
		// TODO: does the WHOAMI make sense as an address?
		m_id.dev_id_s.address = MPU_WHOAMI_9250;
	}

	// @return 0 on success, -errno on failure
	virtual int start();

	// @return 0 on success, -errno on failure
	virtual int stop();

protected:
	virtual void _measure();
	virtual int _publish(struct imu_sensor_data &data);

private:
	// @returns 0 on success, -errno on failure
	int mpu9250_init();
};

}; // namespace DriverFramework
