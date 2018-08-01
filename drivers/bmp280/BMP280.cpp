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
#ifdef __DF_QURT
#include "dev_fs_lib_i2c.h"
#endif

#define BMP280_REG_ID 0xD0
#define BMP280_REG_CTRL_MEAS 0xF4
#define BMP280_REG_CONFIG 0xF5
#define BMP280_REG_PRESS_MSB 0xF7

#define BMP280_ID 0x58

#define BMP280_BITS_CTRL_MEAS_OVERSAMPLING_TEMP2X     0b01000000
#define BMP280_BITS_CTRL_MEAS_OVERSAMPLING_PRESSURE8X 0b00010000
#define BMP280_BITS_CTRL_MEAS_POWER_MODE_NORMAL	      0b00000011

#define BMP280_BITS_CONFIG_STANDBY_0MS5	0b00000000
#define BMP280_BITS_CONFIG_FILTER_OFF	0b00000000
#define BMP280_BITS_CONFIG_SPI_OFF	0b00000000

#if defined(__DF_BBBLUE)
#define BMP280_FILTER_ORDER			2
#define BMP280_FILTER_CUTOFF_FREQ	2.0f
                                    // 2rad/s, about 0.3hz
#define BMP280_FILTER_CHECK_HZ		25
#define	BMP280_FILTER_DT			1.0f/BMP280_FILTER_CHECK_HZ
#endif

using namespace DriverFramework;

int BMP280::set_i2c_slave_config()
{
#if defined(__BARO_USE_SPI)
	return -1;
#else
  #ifdef __DF_BBBLUE
	int result = rc_i2c_set_device_address(m_bus_num, BMP280_SLAVE_ADDRESS);
  #else
	int result = _setSlaveConfig(BMP280_SLAVE_ADDRESS,
				 BMP280_BUS_FREQUENCY_IN_KHZ,
				 BMP280_TRANSFER_TIMEOUT_IN_USECS);
  #endif

	if (result < 0) {
		DF_LOG_ERR("Could not set slave config, result: %d", result);
	}

	return result;
#endif
}

// convertPressure must be called after convertTemperature
// as convertTemperature sets m_sensor_data.t_fine
uint32_t BMP280::convertPressure(int64_t adc_P)
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

	return (uint32_t)p;
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

int BMP280::bmp280_init()
{
	/* Zero the struct */

	m_sensor_data.pressure_pa = 0.0f;
	m_sensor_data.temperature_c = 0.0f;
	m_sensor_data.last_read_time_usec = 0;
	m_sensor_data.read_counter = 0;
	m_sensor_data.error_counter = 0;

#if defined(__DF_BBBLUE_USE_RC_BMP280_IMP)
	// use an implementation with Robotics Cape library and a second order
	// low pass filter to filter out the high frequency noise inherent in MEMS barometers.
	m_low_pass_filter = rc_filter_empty();

	if (rc_bmp_init(BMP_OVERSAMPLE_16, BMP_FILTER_OFF) < 0) {
		DF_LOG_ERR("ERROR: rc_initialize_barometer failed");
		return -1;
	}

	// create the lowpass filter and prefill with current altitude
	if (rc_filter_butterworth_lowpass(&m_low_pass_filter, BMP280_FILTER_ORDER,
			                   BMP280_FILTER_DT, BMP280_FILTER_CUTOFF_FREQ)) {
		DF_LOG_ERR("ERROR: failed to make butterworth filter");
		return -1;
	}

	rc_bmp_data_t data;
	if (rc_bmp_read(&data)) {
		DF_LOG_ERR("ERROR: rc_bmp_read failed");
		return -1;
	}

	// prefill low pass filter
	rc_filter_prefill_inputs (&m_low_pass_filter, data.alt_m);
	rc_filter_prefill_outputs(&m_low_pass_filter, data.alt_m);

#else
	int result;
	uint8_t sensor_id;

	/* Read the ID of the BMP280 sensor to confirm it's presence. */
	result = _readReg(BMP280_REG_ID, &sensor_id, sizeof(sensor_id));

	if (result != 0) {
		DF_LOG_ERR("error: unable to communicate with the bmp280 pressure sensor");
		return -EIO;
	}

	if (sensor_id != BMP280_ID) {
		DF_LOG_ERR("BMP280 sensor ID returned 0x%x instead of 0x%x", sensor_id, BMP280_ID);
		return -1;
	}

	/* Load and display the internal calibration values. */
	result = loadCalibration();

	if (result != 0) {
		DF_LOG_ERR("error: unable to complete initialization of the bmp280 pressure sensor");
		return -EIO;
	}

	uint8_t ctrl_meas = (BMP280_BITS_CTRL_MEAS_OVERSAMPLING_TEMP2X |
			     BMP280_BITS_CTRL_MEAS_OVERSAMPLING_PRESSURE8X |
			     BMP280_BITS_CTRL_MEAS_POWER_MODE_NORMAL);

	result = _writeReg(BMP280_REG_CTRL_MEAS, &ctrl_meas, sizeof(ctrl_meas));

	if (result != 0) {
		DF_LOG_ERR("error: sensor configuration failed");
		return -EIO;
	}

	uint8_t config = (BMP280_BITS_CONFIG_STANDBY_0MS5 |
			  BMP280_BITS_CONFIG_FILTER_OFF |
			  BMP280_BITS_CONFIG_SPI_OFF);

	result = _writeReg(0xF5, &config, sizeof(config));

	if (result != 0) {
		DF_LOG_ERR("error: BMP280 sensor configuration failed");
		return -EIO;
	}
#endif

	DF_LOG_INFO("BMP280 sensor configuration succeeded");

	usleep(1000);
	return 0;
}

int BMP280::start()
{
#if defined(__DF_BBBLUE)
	// on BBBLUE, MPU9250 and BMP280 are on the same I2C bus
	pthread_mutex_lock(&_mutex_shared_i2c_2_bus);
		int ret = _start();
	pthread_mutex_unlock(&_mutex_shared_i2c_2_bus);
	return ret;
#else
	return _start();
#endif
}

int BMP280::_start()
{
	rc_init();

	int result = I2CDevObj::start();

	if (result != 0) {
		DF_LOG_ERR("error: could not start DevObj");
		goto exit;
	}

	/* Configure the I2C bus parameters for the pressure sensor. */

	result = set_i2c_slave_config();

	if (result != 0) {
		DF_LOG_ERR("I2C slave configuration failed");
		goto exit;
	}

	/* Initialize the pressure sensor for active and continuous operation. */
	result = bmp280_init();

	if (result != 0) {
		DF_LOG_ERR("error: pressure sensor initialization failed, sensor read thread not started");
		goto exit;
	}


	result = DevObj::start();

	if (result != 0) {
		DF_LOG_ERR("error: could not start DevObj");
		goto exit;
	}

exit:
	return result;
}

int BMP280::stop()
{
	int result = DevObj::stop();

	if (result != 0) {
		DF_LOG_ERR("DevObj stop failed");
		return result;
	}

	return 0;
}

void BMP280::_measure()
{
#if defined(__DF_BBBLUE)
	pthread_mutex_lock(&_mutex_shared_i2c_2_bus);
		int busInUse = rc_i2c_get_lock(2);
		if (!busInUse) {
			rc_i2c_lock_bus(2);

			#if defined(__DF_BBBLUE_USE_RC_BMP280_IMP)
				_measureDataRC();
			#else
				_measureData();
			#endif

			rc_i2c_unlock_bus(2);
		}
	pthread_mutex_unlock(&_mutex_shared_i2c_2_bus);
#else
	_measureData();
#endif
}

void BMP280::_measureData()
{
	uint8_t pdata[BMP280_MAX_LEN_SENSOR_DATA_BUFFER_IN_BYTES];
	memset(pdata, 0, BMP280_MAX_LEN_SENSOR_DATA_BUFFER_IN_BYTES);

	/* Configure the I2C bus parameters for the pressure sensor every time */
	if (set_i2c_slave_config()) {
		DF_LOG_ERR("BMP280: I2C slave configuration failed");
		return;
	}

	/* Read the data from the pressure sensor. */
	int result = _readReg(BMP280_REG_PRESS_MSB, pdata, BMP280_MAX_LEN_SENSOR_DATA_BUFFER_IN_BYTES);

	if (result < 0) {
		DF_LOG_ERR("error: reading I2C bus failed");
		return;
	}

	// TODO: add endianness defines and do this nicer
	uint32_t pressure_from_sensor = 0;
	pressure_from_sensor |= pdata[0];
	pressure_from_sensor <<= 8;
	pressure_from_sensor |= pdata[1];
	pressure_from_sensor <<= 8;
	pressure_from_sensor |= pdata[2] & 0b11110000;
	pressure_from_sensor >>= 4;

	uint32_t temperature_from_sensor = 0;
	temperature_from_sensor |= pdata[3];
	temperature_from_sensor <<= 8;
	temperature_from_sensor |= pdata[4];
	temperature_from_sensor <<= 8;
	temperature_from_sensor |= pdata[5] & 0b11110000;
	temperature_from_sensor >>= 4;

	/*
	 * Process the raw sensor values, being certain to process temperature first
	 * to obtain the latest temperature reading.
	 */

	m_sensor_data.temperature_c = convertTemperature(temperature_from_sensor) / 100.0;
	m_sensor_data.pressure_pa = convertPressure(pressure_from_sensor) / 256.0;
	m_sensor_data.last_read_time_usec = DriverFramework::offsetTime();
	m_sensor_data.read_counter++;

	_publish(m_sensor_data);

}

void BMP280::_measureDataRC()
{
#if defined(__DF_BBBLUE_USE_RC_BMP280_IMP)
	rc_bmp_data_t data;
	if (rc_bmp_read(&data)) {
		DF_LOG_ERR("BMP280: Can't read Barometer");
		return;
	}

	m_sensor_data.temperature_c 		= data.temp_c;
	m_sensor_data.pressure_pa 			= data.pressure_pa;
	m_sensor_data.altitude_m 			= rc_filter_march(&m_low_pass_filter, data.alt_m);
	m_sensor_data.last_read_time_usec 	= DriverFramework::offsetTime();
	m_sensor_data.read_counter++;

	//DF_LOG_DEBUG("BMP280::_measureDataRC: pressure: %9.2f pa, unfiltered alt: %9.2f m, altitude: %9.2f m, temp: %7.2f C",
	//		    m_sensor_data.pressure_pa, alt_m, m_sensor_data.altitude_m, m_sensor_data.temperature_c);

	_publish(m_sensor_data);
#endif
}

