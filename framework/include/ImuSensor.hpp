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

#if defined(__DF_BBBLUE)
#define IMU_DEVICE_PATH "/dev/i2c-2"
#elif defined(__DF_BEBOP)
#define IMU_DEVICE_PATH "/dev/i2c-mpu6050"
#else
#define IMU_DEVICE_PATH "/dev/iic-2"
#endif

#else  // !defined(__IMU_USE_I2C)
#include "SPIDevObj.hpp"

#if defined(__DF_QURT)
#include "dev_fs_lib_spi.h"
#define IMU_DEVICE_PATH "/dev/spi-1"
#elif defined(__DF_BEBOP)
#define IMU_DEVICE_PATH "/dev/i2c-mpu6050"
#elif defined(__DF_RPI)
#define IMU_DEVICE_PATH "/dev/spidev0.1"
#elif defined(__DF_EDISON)
#define IMU_DEVICE_PATH "/dev/spidev5.1"
#elif defined(__DF_OCPOC)
#define IMU_DEVICE_PATH "/dev/spidev1.0"
#else
#define IMU_DEVICE_PATH "/dev/spidev0.0"
#endif

#endif // __IMU_USE_I2C

#if defined(__DF_RPI)
#include <linux/spi/spidev.h>
#define IMU_DEVICE_ACC_GYRO "/dev/spidev0.3"
#define IMU_DEVICE_MAG "/dev/spidev0.2"
#elif defined(__DF_RPI_SINGLE)
#define IMU_DEVICE_ACC_GYRO "/dev/spidev0.1"
#define IMU_DEVICE_MAG "/dev/spidev0.1"

#elif defined(__DF_BBBLUE)
#define IMU_DEVICE_ACC_GYRO "/dev/i2c-2"
#define IMU_DEVICE_MAG      "/dev/i2c-2"

#else
#define IMU_DEVICE_ACC_GYRO ""
#define IMU_DEVICE_MAG      ""
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

void printImuValues(struct imu_sensor_data &data);

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

	virtual ~ImuSensor() = default;

protected:
	virtual void _measure() = 0;

	struct imu_sensor_data 		m_sensor_data;
	bool						m_mag_enabled;
};

} // namespace DriverFramework
