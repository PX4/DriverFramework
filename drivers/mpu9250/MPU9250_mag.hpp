/****************************************************************************
 *
 *   Copyright (C) 2016 James Y. Wilson. All rights reserved.
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

#include "MPU9250.hpp"

namespace DriverFramework {

// Forward reference:
class MPU9250;

// Magnetometer device ID
#define MPU9250_AKM_DEV_ID		0x48

// Magnetometer device address
#define MPU9250_AK8963_I2C_ADDR  0x0C
#define MPU9250_AK8963_I2C_READ  0x80
#define MPU9250_AK8963_I2C_WRITE 0x00

// MPU9250 Magnetometer Register Addresses: Defines only the register addresses
// used in MPU9250 driver.
#define MPU9250_MAG_REG_WIA		0x00
#define MPU9250_MAG_REG_ST1		0x02
#define MPU9250_MAG_REG_DATA	0x03
#define MPU9250_MAG_REG_HXL     0x03
#define MPU9250_MAG_REG_ST2		0x09
#define MPU9250_MAG_REG_CNTL1	0x0a
#define MPU9250_MAG_REG_ASAX	0x10
#define MPU9250_MAG_REG_ASAY    0x11
#define MPU9250_MAG_REG_ASAZ    0x12

// Bit definitions for the magnetometer registers
#define BIT_MAG_CNTL1_MODE_POWER_DOWN 				 0x0
#define BIT_MAG_CNTL1_MODE_SINGLE_MEASURE_MODE 		 0x1
#define BIT_MAG_CNTL1_MODE_CONTINUOUS_MEASURE_MODE_1 0x2
#define BIT_MAG_CNTL1_MODE_CONTINUOUS_MEASURE_MODE_2 0x6
#define BIT_MAG_CNTL1_FUSE_ROM_ACCESS_MODE 		 	 0xF
#define BIT_MAG_CNTL1_16_BITS 						 0x10
#define BIT_MAG_HOFL                                 0x08

#define MAG_ERROR_DATA_OVERFLOW                      -3

enum mag_sample_rate_e
{
	MPU9250_MAG_SAMPLE_RATE_8HZ = 0,
	MPU9250_MAG_SAMPLE_RATE_100HZ = 1,
	NUM_MPU9250_MAG_SAMPLE_RATES
};

class MPU9250_mag
{
public:
	MPU9250_mag(MPU9250 &imu, enum mag_sample_rate_e sample_rate) :
			_mag_initialized(false), _sample_rate(sample_rate), _imu(imu)
	{
	};

	/// @brief
	/// Called to initialize the magnetometer connection via the
	/// internal I2C bus of the sensor.  The gyro and accelerometer
	/// must have been previously configured.
	/// @return
	/// - 0 on success,
	/// - -errno on failure
	int initialize(int output_data_rate_in_hz);

	/// @brief
	/// Reads the sensitivity values contained in the FUSE memory of the mag and
	/// generates values used internally to process mag measurements.
	/// @return
	/// - 0 on success, -
	/// errno on failure
	int get_sensitivity_adjustment(void);

	/// @brief
	/// Verifies the presence of the mag on the internal I2C bus, by querying
	/// for the known device ID.
	/// @return
	/// - 0 on success
	/// - -errno on failure
	int detect(void);

	/// @brief
	/// Writes a value to the specified IMU register, and verifies that it was
	/// successfully written by attempting to read it back.
	/// @return
	/// - 0 on success
	/// - -errno on failure
	int write_imu_reg_verified(int reg, uint8_t val, uint8_t mask);

	/// @brief
	/// Reads the value of the specified magnetometer register and returns the
	/// register value in the "val" parameter.
	/// @return
	/// - 0 on success
	/// - -errno on failure
	int read_reg(uint8_t reg, uint8_t *val);

	/// @brief
	/// Writes the value passed in to the specified magnetometer register.
	/// @return
	/// - 0 on success
	/// - -errno on failure
	int write_reg(uint8_t reg, uint8_t val);

	/// @brief
	/// Process the data passed in to generate mag values in Gauss units.
	/// The values passed in are modified and returned in the same parameter.
	/// @return
	/// - 0 on success
	/// - -errno on failure
	int process(struct mag_data &data);

protected:
	/// @brief
	/// Used internally to perform a complete mag initialization.  Called
	/// multiple times by the initialize() function if the first initialization
	/// attempt fails.
	/// @param
	/// output_data_rate_in_hz
	/// The rate at which the sensor produces new IMU data (accel, gyro, and temperature data)
	/// @return
	/// - 0 on success
	/// - -errno on failure
	int _initialize(int output_data_rate_in_hz);

	/// @brief
	/// Convert the magnetometer sample rate enum to an equivalent number in Hz.
	/// @return
	/// - 0 on success
	/// - -errno on failure
	int _convert_sample_rate_enum_to_hz(enum mag_sample_rate_e sample_rate);

private:
	float _mag_range_ga;
	float _mag_scaling;
	int _mag_sens_adj[3];
	float _mag_initialized;
	mag_sample_rate_e _sample_rate;

	// Internal reference to the MPU9250 object that instantiated this mag class.
	MPU9250 &_imu;

	int _initialize();
};

};
