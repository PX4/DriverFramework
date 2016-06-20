/****************************************************************************
 *
 *   Copyright (C) 2015 Mark Charlebois. All rights reserved.
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

#include "BaroSensor.hpp"

namespace DriverFramework
{

struct ms5607_sensor_calibration {
	uint16_t factory_setup;
	uint16_t c1_pressure_sens;
	uint16_t c2_pressure_offset;
	uint16_t c3_temp_coeff_pres_sens;
	uint16_t c4_temp_coeff_pres_offset;
	uint16_t c5_reference_temp;
	uint16_t c6_temp_coeff_temp;
	uint16_t serial_and_crc;
};

struct ms5607_sensor_measurement {
	int32_t temperature_cc; // Temperature with 0.01 DegC resolution 2356 = 23.56 DegC
	int64_t off; // Offset at actual temperature
	int64_t sens; // Sensitivity at actual temperature
	int32_t pressure_mbar; // Temperature compensated pressure with 0.01 mbar resolution
};

#define BARO_DEVICE_PATH "/dev/i2c-ms5607"

// update frequency is 50 Hz (44.4-51.3Hz ) at 8x oversampling
#define MS5607_MEASURE_INTERVAL_US 20000 // microseconds
#define MS5607_CONVERSION_INTERVAL_US 10000 /*microseconds */

#define MS5607_BUS_FREQUENCY_IN_KHZ 400
#define MS5607_TRANSFER_TIMEOUT_IN_USECS 9000

#define MS5607_MAX_LEN_SENSOR_DATA_BUFFER_IN_BYTES 6
#define MS5607_MAX_LEN_CALIB_VALUES 16

#define DRV_DF_DEVTYPE_MS5607 0x42

#define MS5607_SLAVE_ADDRESS 0x77       /* 7-bit slave address */

class MS5607 : public BaroSensor
{
public:
	MS5607(const char *device_path) :
		BaroSensor(device_path, MS5607_MEASURE_INTERVAL_US)
	{
		m_id.dev_id_s.devtype = DRV_DF_DEVTYPE_MS5607;
		m_id.dev_id_s.address = MS5607_SLAVE_ADDRESS;
	}

	// @return 0 on success, -errno on failure
	virtual int start();

	// @return 0 on success, -errno on failure
	virtual int stop();

protected:
	virtual void _measure();
	virtual int _publish(struct baro_sensor_data &data);

private:
	// Returns pressure in Pa as unsigned 32 bit integer
	// Output value of “24674867” represents
	// 24674867/100 = 246748.67 Pa = 24674867 hPa
	int64_t convertPressure(int64_t adc_pressure);

	// Returns temperature in DegC, resolution is 0.01 DegC
	// Output value of “5123” equals 51.23 DegC
	int32_t convertTemperature(int32_t adc_temperature);

	int loadCalibration();

	// Request to convert pressure or temperature data
	int _request(uint8_t phase);
	// Read out the requested sensor data
	int _collect(uint32_t *raw);

	bool crc4(uint16_t *n_prom);

	// returns 0 on success, -errno on failure
	int ms5607_init();

	// Send reset to device
	int reset();

	struct ms5607_sensor_calibration 	m_sensor_calibration;
	struct ms5607_sensor_measurement m_raw_sensor_convertion;;
};

}; // namespace DriverFramework
