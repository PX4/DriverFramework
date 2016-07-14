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

#include <stdint.h>
#include "SyncObj.hpp"

#if defined(__IMU_USE_I2C)
#include "I2CDevObj.hpp"
#else
#include "SPIDevObj.hpp"
#endif

#if defined(__QURT)
#include "dev_fs_lib_spi.h"
#define IMU_DEVICE_PATH "/dev/spi-1"
#elif defined(__BEBOP)
#define IMU_DEVICE_PATH "/dev/i2c-mpu6050"
#elif defined(__RPI)
#define IMU_DEVICE_PATH "/dev/spidev0.1"
#elif defined(__EDISON)
#define IMU_DEVICE_PATH "/dev/spidev5.1"
#else
#define IMU_DEVICE_PATH "/dev/spidev0.0"
#endif

#if defined(__RPI)
#include <linux/spi/spidev.h>
#define IMU_DEVICE_ACC_GYRO "/dev/spidev0.3"
#define IMU_DEVICE_MAG "/dev/spidev0.2"
#else
#define IMU_DEVICE_ACC_GYRO ""
#define IMU_DEVICE_MAG ""
#endif

#define IMU_CLASS_PATH  "/dev/imu"

namespace DriverFramework
{

/**
 * The sensor independent data structure containing IMU values.
 */
struct imu_sensor_data {
	float		accel_m_s2_x;
	float		accel_m_s2_y;
	float		accel_m_s2_z;
	float		gyro_rad_s_x;
	float		gyro_rad_s_y;
	float		gyro_rad_s_z;
	float		mag_ga_x;
	float       	mag_ga_y;
	float       	mag_ga_z;
	float		temp_c;
	uint64_t	read_counter;
	uint64_t	error_counter;
	uint64_t	fifo_overflow_counter;
	uint64_t	fifo_corruption_counter;
	uint64_t	gyro_range_hit_counter;
	uint64_t	accel_range_hit_counter;
	uint64_t	mag_fifo_overflow_counter;
	unsigned	fifo_sample_interval_us;
	bool		is_last_fifo_sample;
};

#if defined(__IMU_USE_I2C)
class ImuSensor : public I2CDevObj
#else
class ImuSensor : public SPIDevObj
#endif
{
public:
	ImuSensor(const char *device_path, unsigned int sample_interval_usec, bool mag_enabled = false) :
#if defined(__IMU_USE_I2C)
		I2CDevObj("ImuSensor", device_path, IMU_CLASS_PATH, sample_interval_usec),
#else
		SPIDevObj("ImuSensor", device_path, IMU_CLASS_PATH, sample_interval_usec),
#endif
		m_mag_enabled(mag_enabled)
	{}

	~ImuSensor() {}

	static int getSensorData(DevHandle &h, struct imu_sensor_data &out_data, bool is_new_data_required)
	{
		ImuSensor *me = DevMgr::getDevObjByHandle<ImuSensor>(h);
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

	static void printImuValues(DevHandle &h, struct imu_sensor_data &data)
	{
		ImuSensor *me = DevMgr::getDevObjByHandle<ImuSensor>(h);

		if (me != nullptr) {
			DF_LOG_INFO("IMU: accel: [%.2f, %.2f, %.2f] m/s^2",
				    (double)data.accel_m_s2_x,
				    (double)data.accel_m_s2_y,
				    (double)data.accel_m_s2_z);
			DF_LOG_INFO("     gyro:  [%.2f, %.2f, %.2f] rad/s",
				    (double)data.gyro_rad_s_x,
				    (double)data.gyro_rad_s_y,
				    (double)data.gyro_rad_s_z);

			if (me->m_mag_enabled) {
				DF_LOG_INFO("     mag:  [%.6f, %.6f, %.6f] ga",
					    (double)data.mag_ga_x,
					    (double)data.mag_ga_y,
					    (double)data.mag_ga_z);
			}

			DF_LOG_INFO("     temp:  %.2f C",
				    (double)data.temp_c);
		}
	}

protected:
	virtual void _measure() = 0;

	virtual int _publish(struct imu_sensor_data &data)
	{
		return -1;
	};

	struct imu_sensor_data 		m_sensor_data;
	bool						m_mag_enabled;
	SyncObj 					m_synchronize;
};

} // namespace DriverFramework
