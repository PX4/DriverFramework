/****************************************************************************
 *
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

#pragma once

#define __IMU_USE_I2C
#include "ImuSensor.hpp"

#define DRV_DF_DEVTYPE_MPU6050 0x45

#define MPU_WHOAMI_6050		 0x68
#define MPU6050_SLAVE_ADDRESS 0x68       /* 7-bit slave address */

// update frequency 1000 Hz
#define MPU6050_MEASURE_INTERVAL_US 1000

#define MPU6050_BUS_FREQUENCY_IN_KHZ 400
#define MPU6050_TRANSFER_TIMEOUT_IN_USECS 900

namespace DriverFramework
{

class MPU6050: public ImuSensor
{
public:
	MPU6050(const char *device_path) :
		ImuSensor(device_path, MPU6050_MEASURE_INTERVAL_US, false), // false = sensor has no mag
		_last_temp_c(0.0f),
		_temp_initialized(false),
		_packets_per_cycle_filtered(1.0)
	{
		m_id.dev_id_s.devtype = DRV_DF_DEVTYPE_MPU6050;
		m_id.dev_id_s.address = MPU6050_SLAVE_ADDRESS;
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
	int mpu6050_init();

	// @returns 0 on success, -errno on failure
	int mpu6050_deinit();

	// @return the number of FIFO bytes to collect
	int get_fifo_count();

	void reset_fifo();

	float _last_temp_c;
	bool _temp_initialized;
	float _packets_per_cycle_filtered;
};

}
// namespace DriverFramework

