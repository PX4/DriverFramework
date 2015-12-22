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

#include <string.h>
#include "DriverFramework.hpp"
#include "BMP280.hpp"

using namespace DriverFramework;

// convertPressure must be called after convertTemperature
// as convertTemperature sets m_sensor_data.t_fine
int64_t BMP280::convertPressure(int64_t adc_P)
{
	int64_t var1, var2, p;
	var1 = ((int64_t) m_sensor_data.t_fine) - 128000;
	var2 = var1 * var1 * (int64_t) m_sensor_calibration.dig_P6;
	var2 = var2 + ((var1 * (int64_t) m_sensor_calibration.dig_P5) << 17);
	var2 = var2 + (((int64_t) m_sensor_calibration.dig_P4) << 35);
	var1 = ((var1 * var1 * (int64_t) m_sensor_calibration.dig_P3) >> 8)
			+ ((var1 * (int64_t) m_sensor_calibration.dig_P2) << 12);
	var1 = (((((int64_t) 1) << 47) + var1) * ((int64_t) m_sensor_calibration.dig_P1)) >> 33;
	if (var1 == 0) {
		return 0; // avoid exception caused by division by zero
	}

	p = 1048576 - adc_P;
	p = (((p << 31) - var2) * 3125) / var1;
	var1 = (((int64_t) m_sensor_calibration.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
	var2 = (((int64_t) m_sensor_calibration.dig_P8) * p) >> 19;
	p = ((p + var1 + var2) >> 8) + (((int64_t) m_sensor_calibration.dig_P7) << 4);

	return p;
}

int32_t BMP280::convertTemperature(int32_t adc_T)
{
	int32_t var1, var2, T;
	var1 = ((((adc_T >> 3)
			- ((int32_t) m_sensor_calibration.dig_T1 << 1)))
			* ((int32_t) m_sensor_calibration.dig_T2)) >> 11;
	var2 = (((((adc_T >> 4) - ((int32_t) m_sensor_calibration.dig_T1))
			* ((adc_T >> 4) - ((int32_t) m_sensor_calibration.dig_T1)))
			>> 12) * ((int32_t) m_sensor_calibration.dig_T3)) >> 14;
	m_sensor_data.t_fine = var1 + var2;
	T = (m_sensor_data.t_fine * 5 + 128) >> 8;

	return T;
}

int BMP280::loadCalibration()
{
	int result;
	uint8_t calib_values[BMP280_MAX_LEN_CALIB_VALUES];
	uint16_t *calib_value_ptr = (uint16_t *) &calib_values[0];
	memset(calib_values, 0, BMP280_MAX_LEN_CALIB_VALUES);

	result = _readReg(0x88, calib_values, BMP280_MAX_LEN_CALIB_VALUES);
	if (result != 0) {
		DF_LOG_ERR("error: unable to read sensor calibration values from the sensor");
		return -EIO;
	}

	memcpy(&m_sensor_calibration.dig_T1, calib_value_ptr++, sizeof(uint16_t));
	memcpy(&m_sensor_calibration.dig_T2, calib_value_ptr++, sizeof(uint16_t));
	memcpy(&m_sensor_calibration.dig_T3, calib_value_ptr++, sizeof(uint16_t));
	memcpy(&m_sensor_calibration.dig_P1, calib_value_ptr++, sizeof(uint16_t));
	memcpy(&m_sensor_calibration.dig_P2, calib_value_ptr++, sizeof(uint16_t));
	memcpy(&m_sensor_calibration.dig_P3, calib_value_ptr++, sizeof(uint16_t));
	memcpy(&m_sensor_calibration.dig_P4, calib_value_ptr++, sizeof(uint16_t));
	memcpy(&m_sensor_calibration.dig_P5, calib_value_ptr++, sizeof(uint16_t));
	memcpy(&m_sensor_calibration.dig_P6, calib_value_ptr++, sizeof(uint16_t));
	memcpy(&m_sensor_calibration.dig_P7, calib_value_ptr++, sizeof(uint16_t));
	memcpy(&m_sensor_calibration.dig_P8, calib_value_ptr++, sizeof(uint16_t));
	memcpy(&m_sensor_calibration.dig_P9, calib_value_ptr++, sizeof(uint16_t));

	return 0;
}

int BMP280::bmp280_init() {
	int result;
	uint8_t sensor_id;

	/* Read the ID of the BMP280 sensor to confirm it's presence. */
	result = _readReg(0xD0, &sensor_id, sizeof(sensor_id));
	if (result != 0) {
		DF_LOG_ERR("error: unable to communicate with the bmp280 pressure sensor");
		return -EIO;
	}
	DF_LOG_ERR("BMP280 pressure sensor ID: 0x%X", sensor_id);

	/* Load and display the internal calibration values. */
	result = loadCalibration();
	if (result != 0) {
		DF_LOG_ERR("error: unable to complete initialization of the bmp280 pressure sensor");
		return -EIO;
	}

	// power on, oversampling 2 for temp and 8 for pressure
	uint8_t ctrl_meas = 0b01010011;
	result = _writeReg(0xF4, &ctrl_meas, 1);
	if (result != 0) {
		DF_LOG_ERR("error: sensor configuration failed");
		return -EIO;
	}
	DF_LOG_ERR("sensor configuration succeeded");

	uint8_t config = 0b00000000;
	result = _writeReg(0xF5, &config, 1);
	if (result != 0) {
		DF_LOG_ERR("error: additional sensor configuration failed");
		return -EIO;
	}
	DF_LOG_ERR("additional sensor configuration succeeded");

	usleep(10000);
	return 0;
}

int BMP280::start()
{
	int result = 0;

	/* Open the device path specified in the class initialization */
	if (devOpen(0) < 0) {
		DF_LOG_ERR("Unable to open the device path: %s", m_dev_path);
		result = -1;
		goto exit;
	}

	/* Configure the I2C bus parameters for the pressure sensor. */
	memset(&m_slave_config, 0, sizeof(m_slave_config));
	m_slave_config.slave_address = BMP280_SLAVE_ADDRESS;
	m_slave_config.bus_frequency_in_khz = BMP280_BUS_FREQUENCY_IN_KHZ;
	m_slave_config.byte_transer_timeout_in_usecs = BMP280_TRANSFER_TIMEOUT_IN_USECS;
	if (devIOCTL(I2C_IOCTL_SLAVE, reinterpret_cast<unsigned long>(&m_slave_config)) != 0) {
		DF_LOG_ERR("unable to open the device path: %s", m_dev_path);
		result = -1;
		goto exit;
	}

	/* Initialize the pressure sensor for active and continuous operation. */
	result = bmp280_init();
	if (result != 0) {
		DF_LOG_ERR("error: pressure sensor initialization failed, sensor read thread not started");
		goto exit;
	}

exit:
	if (result != 0) {
		devClose();
	}
	else {
		result = I2CDevObj::start();
	}

	return result;
}

void BMP280::_measure(void)
{
	int bmp_ret_code;
	uint32_t pressure_from_sensor;
	uint32_t temperature_from_sensor;
	float pressure_in_pa;
	float temperature_in_c;
	uint8_t pdata[BMP280_MAX_LEN_SENSOR_DATA_BUFFER_IN_BYTES];
	memset(pdata, 0, BMP280_MAX_LEN_SENSOR_DATA_BUFFER_IN_BYTES);

	/* Read the data from the pressure sensor. */
	bmp_ret_code = _readReg(0xF7, pdata, BMP280_MAX_LEN_SENSOR_DATA_BUFFER_IN_BYTES);
	if (bmp_ret_code < 0) {
		DF_LOG_ERR("error: reading I2C bus failed");
		return;
	}

	pressure_from_sensor = 0;
	pressure_from_sensor |= pdata[0];
	pressure_from_sensor <<= 8;
	pressure_from_sensor |= pdata[1];
	pressure_from_sensor <<= 8;
	pressure_from_sensor |= pdata[2] & 0b11110000;
	pressure_from_sensor >>= 4;

	temperature_from_sensor = 0;
	temperature_from_sensor |= pdata[3];
	temperature_from_sensor <<= 8;
	temperature_from_sensor |= pdata[4];
	temperature_from_sensor <<= 8;
	temperature_from_sensor |= pdata[5] & 0b11110000;
	temperature_from_sensor >>= 4;
	DF_LOG_DEBUG("raw sensor values: pressure: %u, temperature: %u",
			pressure_from_sensor, temperature_from_sensor);

	/*
	 * Process the raw sensor values, being certain to process temperature first
	 * to obtain the latest temperature reading.
	 */
	temperature_in_c = convertTemperature(temperature_from_sensor);
	pressure_in_pa = convertPressure(pressure_from_sensor);

	m_synchronize.lock();

	m_sensor_data.pressure_in_pa = pressure_in_pa / 256.0;
	m_sensor_data.temperature_in_c = temperature_in_c / 100.0;
	m_sensor_data.last_read_time_in_usecs = DriverFramework::offsetTime();
	m_sensor_data.sensor_read_counter++;

	m_synchronize.signal();
	m_synchronize.unlock();
}
