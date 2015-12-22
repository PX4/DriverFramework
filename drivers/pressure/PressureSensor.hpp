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

struct bmp280_sensor_calibration
{
	uint16_t dig_T1;
	uint16_t dig_P1;
	int16_t dig_T2;
	int16_t dig_T3;
	int16_t dig_P2;
	int16_t dig_P3;
	int16_t dig_P4;
	int16_t dig_P5;
	int16_t dig_P6;
	int16_t dig_P7;
	int16_t dig_P8;
	int16_t dig_P9;
};

#define BMP280_SLAVE_ADDRESS 0b1110110       /* 7-bit slave address */
#define BMP280_BUS_FREQUENCY_IN_KHZ 400
#define BMP280_TRANSFER_TIMEOUT_IN_USECS 9000

#define BMP280_MAX_LEN_SENSOR_DATA_BUFFER_IN_BYTES 6
#define BMP280_MAX_LEN_CALIB_VALUES 26

typedef int32_t  BMP280_S32_t;
typedef int64_t  BMP280_S64_t;
typedef uint32_t BMP280_U32_t;

class PressureSensor : public I2CDevObj
{
public:
	PressureSensor(const char *device_path) :
		I2CDevObj("PressureSensor", device_path, PRESSURE_CLASS_PATH, 1000)
	{}

	void setAltimeter(float altimeter_setting_in_mbars);

	static int getSensorData(DevHandle &h, struct pressure_sensor_data &out_data, bool is_new_data_required);

	virtual int start();

protected:
	virtual void _measure();

private:
	// Returns pressure in Pa as unsigned 32 bit integer in
	// Q24.8 format (24 integer bits and 8 fractional bits)
	// Output value of “24674867” represents
	// 24674867/256 = 96386.2 Pa = 963.862 hPa
	int64_t convertPressure(int64_t adc_pressure);

	// Returns temperature in DegC, resolution is 0.01 DegC
	// Output value of “5123” equals 51.23 DegC
	int32_t convertTemperature(int32_t adc_temperature);

	int loadCalibration();

	// returns 0 on success, -errno on failure
	int bmp280_init();

	struct dspal_i2c_ioctl_slave_config 	m_slave_config;
	struct pressure_sensor_data 		m_sensor_data;
	struct bmp280_sensor_calibration 	m_sensor_calibration;
	float 					m_altimeter_mbars = 0.0;
	SyncObj 				m_synchronize;
	int					m_last_error;
};

}; // namespace DriverFramework
