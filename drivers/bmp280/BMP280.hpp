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

#include "BaroSensor.hpp"

namespace DriverFramework
{

struct bmp280_sensor_calibration {
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


#define BARO_DEVICE_PATH "/dev/iic-3"

// update frequency is 50 Hz (44.4-51.3Hz ) at 8x oversampling
#define BMP280_MEASURE_INTERVAL_US 20000

#define BMP280_BUS_FREQUENCY_IN_KHZ 400
#define BMP280_TRANSFER_TIMEOUT_IN_USECS 9000

#define BMP280_MAX_LEN_SENSOR_DATA_BUFFER_IN_BYTES 6
#define BMP280_MAX_LEN_CALIB_VALUES 26

// TODO: include some common header file (currently in drv_sensor.h).
#define DRV_DF_DEVTYPE_BMP280 0x42

#define BMP280_SLAVE_ADDRESS 0b1110110       /* 7-bit slave address */


class BMP280 : public BaroSensor
{
public:
	BMP280(const char *device_path) :
		BaroSensor(device_path, BMP280_MEASURE_INTERVAL_US)
	{
		m_id.dev_id_s.devtype = DRV_DF_DEVTYPE_BMP280;
		m_id.dev_id_s.address = BMP280_SLAVE_ADDRESS;
	}

	// @return 0 on success, -errno on failure
	virtual int start();

	// @return 0 on success, -errno on failure
	virtual int stop();

protected:
	virtual void _measure();
	virtual int _publish(struct baro_sensor_data &data);

private:
	// Returns pressure in Pa as unsigned 32 bit integer in
	// Q24.8 format (24 integer bits and 8 fractional bits)
	// Output value of “24674867” represents
	// 24674867/256 = 96386.2 Pa = 963.862 hPa
	uint32_t convertPressure(int64_t adc_pressure);

	// Returns temperature in DegC, resolution is 0.01 DegC
	// Output value of “5123” equals 51.23 DegC
	int32_t convertTemperature(int32_t adc_temperature);

	int loadCalibration();

	// returns 0 on success, -errno on failure
	int bmp280_init();

	struct bmp280_sensor_calibration 	m_sensor_calibration;
};

}; // namespace DriverFramework
