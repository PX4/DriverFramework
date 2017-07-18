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

struct ms5611_sensor_calibration {
	uint16_t factory_setup;
	uint16_t c1_pressure_sens;
	uint16_t c2_pressure_offset;
	uint16_t c3_temp_coeff_pres_sens;
	uint16_t c4_temp_coeff_pres_offset;
	uint16_t c5_reference_temp;
	uint16_t c6_temp_coeff_temp;
	uint16_t serial_and_crc;
};

struct ms5611_sensor_measurement {
	int32_t temperature_cc; // Temperature with 0.01 DegC resolution 2356 = 23.56 DegC
	int64_t off; // Offset at actual temperature
	int64_t sens; // Sensitivity at actual temperature
	int32_t pressure_mbar; // Temperature compensated pressure with 0.01 mbar resolution
};

#if defined(__DF_OCPOC)
#define BARO_DEVICE_PATH "/dev/spidev1.1"
#else
#define BARO_DEVICE_PATH "/dev/i2c-1"
#endif

#define MS5611_SCHED_POLICY (SCHED_FIFO)
#define MS5611_SCHED_PRIORITY (sched_get_priority_max(SCHED_FIFO) - 10)
#define MS5611_MEASURE_INTERVAL_US (20000)

#define MS5611_SPI_FREQ_HZ (1000000)

#define MS5611_I2C_FREQ_KHZ (400)
#define MS5611_I2C_TIMEOUT_US (9000)
#define MS5611_I2C_ADDR (0x77) /* 7-bit slave address */

#define MS5611_MAX_LEN_SENSOR_DATA_BUFFER_IN_BYTES 6
#define MS5611_MAX_LEN_CALIB_VALUES 16

#define DRV_DF_DEVTYPE_MS5611 0x45


class MS5611 : public BaroSensor
{
public:
	MS5611(const char *device_path);

	virtual ~MS5611() = default;

	// @return 0 on success, -errno on failure
	virtual int start() override;

	// @return 0 on success, -errno on failure
	virtual int stop() override;

protected:
	virtual void _measure() override;
	virtual int _publish(struct baro_sensor_data &data) = 0;

	// Returns pressure in Pa as unsigned 32 bit integer
	// Output value of “24674867” represents
	// 24674867/100 = 246748.67 Pa = 24674867 hPa
	int64_t convertPressure(int64_t adc_pressure);

	// Returns temperature in DegC, resolution is 0.01 DegC
	// Output value of “5123” equals 51.23 DegC
	virtual int32_t convertTemperature(int32_t adc_temperature);

	int loadCalibration();

	// Request to convert pressure or temperature data
	int _request(uint8_t phase);
	// Read out the requested sensor data
	int _collect(uint32_t &raw);

	bool crc4(uint16_t *n_prom);

	// Send reset to device
	int reset();

	struct ms5611_sensor_calibration	m_sensor_calibration;
	struct ms5611_sensor_measurement m_raw_sensor_convertion;

private:
	static void* threadFunc(void *arg);

private:
	pthread_t _thread_id;
	bool _started;

};

}; // namespace DriverFramework
