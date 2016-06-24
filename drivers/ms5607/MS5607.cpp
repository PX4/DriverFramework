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

#include <string.h>
#include "DriverFramework.hpp"
#include "MS5607.hpp"

using namespace DriverFramework;

#define ADDR_RESET_CMD				0x1E	/* write to this address to reset chip */
#define ADDR_CMD_CONVERT_D1_OSR256		0x40	/* write to this address to start pressure conversion */
#define ADDR_CMD_CONVERT_D1_OSR512		0x42	/* write to this address to start pressure conversion */
#define ADDR_CMD_CONVERT_D1_OSR1024		0x44	/* write to this address to start pressure conversion */
#define ADDR_CMD_CONVERT_D1_OSR2048		0x46	/* write to this address to start pressure conversion */
#define ADDR_CMD_CONVERT_D1_OSR4096		0x48	/* write to this address to start pressure conversion */
#define ADDR_CMD_CONVERT_D2_OSR256		0x50	/* write to this address to start temperature conversion */
#define ADDR_CMD_CONVERT_D2_OSR512		0x52	/* write to this address to start temperature conversion */
#define ADDR_CMD_CONVERT_D2_OSR1024		0x54	/* write to this address to start temperature conversion */
#define ADDR_CMD_CONVERT_D2_OSR2048		0x56	/* write to this address to start temperature conversion */
#define ADDR_CMD_CONVERT_D2_OSR4096		0x58	/* write to this address to start temperature conversion */
#define ADDR_CMD_CONVERT_D1   ADDR_CMD_CONVERT_D1_OSR1024
#define ADDR_CMD_CONVERT_D2   ADDR_CMD_CONVERT_D2_OSR1024

#define ADDR_CMD_ADC_READ     0x00
#define ADDR_PROM_SETUP       0xA0  /* address of 8x 2 bytes factory and calibration data */

#define POW2(_x) ((_x) * (_x))

// convertPressure must be called after convertTemperature
// as convertTemperature sets m_raw_sensor_convertion values
int64_t MS5607::convertPressure(int64_t adc_P)
{
	// Conversion from the datasheet
	int64_t p = (((adc_P * m_raw_sensor_convertion.sens) >> 21) - m_raw_sensor_convertion.off) >> 15;
	m_raw_sensor_convertion.pressure_mbar = p;
	return p;
}

int32_t MS5607::convertTemperature(int32_t adc_T)
{
	// Conversion from the datasheet
	/* temperature offset (in ADC units) */
	int32_t dT = adc_T - ((int32_t)m_sensor_calibration.c5_reference_temp << 8);

	int64_t sens = ((int64_t)m_sensor_calibration.c1_pressure_sens << 16)
		       + (((int64_t)m_sensor_calibration.c3_temp_coeff_pres_sens * dT) >> 7);

	int64_t off = ((int64_t)m_sensor_calibration.c2_pressure_offset << 17)
		      + (((int64_t)m_sensor_calibration.c4_temp_coeff_pres_offset * dT) >> 6);

	/* absolute temperature in centidegrees - note intermediate value is outside 32-bit range */
	int32_t temp =  2000 + (int32_t)(((int64_t)dT * m_sensor_calibration.c6_temp_coeff_temp) >> 23);

	/* temperature compensation */
	if (temp < 2000) {
		int32_t t2 = POW2(dT) >> 31;

		int64_t f = POW2((int64_t)temp - 2000);
		int64_t off2 = 61 * f >> 4;
		int64_t sens2 = 2 * f;

		if (temp < -1500) {
			int64_t f2 = POW2(temp + 1500);
			off2 += 15 * f2;
			sens2 += 8 * f2;
		}

		temp -= t2;
		sens -= sens2;
		off -= off2;
	}

	m_raw_sensor_convertion.temperature_cc = temp;
	m_raw_sensor_convertion.sens = sens;
	m_raw_sensor_convertion.off = off;
	return temp;
}

/**
* MS5611 crc4 cribbed from the datasheet
*/
bool MS5607::crc4(uint16_t *n_prom)
{
	int16_t cnt;
	uint16_t n_rem;
	uint16_t crc_read;
	uint8_t n_bit;

	n_rem = 0x00;

	/* save the read crc */
	crc_read = n_prom[7];

	/* remove CRC byte */
	n_prom[7] = (0xFF00 & (n_prom[7]));

	for (cnt = 0; cnt < 16; cnt++) {
		/* uneven bytes */
		if (cnt & 1) {
			n_rem ^= (uint8_t)((n_prom[cnt >> 1]) & 0x00FF);

		} else {
			n_rem ^= (uint8_t)(n_prom[cnt >> 1] >> 8);
		}

		for (n_bit = 8; n_bit > 0; n_bit--) {
			if (n_rem & 0x8000) {
				n_rem = (n_rem << 1) ^ 0x3000;

			} else {
				n_rem = (n_rem << 1);
			}
		}
	}

	/* final 4 bit remainder is CRC value */
	n_rem = (0x000F & (n_rem >> 12));
	n_prom[7] = crc_read;

	/* return true if CRCs match */
	return (0x000F & crc_read) == (n_rem ^ 0x00);
}


int MS5607::loadCalibration()
{
	// Wait for PROM contents to be in the device (2.8 ms), in case we are called
	// immediatelly after reset.
	usleep(3000);

	uint8_t last_val = 0;
	bool bits_stuck = true;

	uint8_t prom_buf[2];
	union {
		uint8_t b[2];
		uint16_t w;
	} cvt;

	for (int i = 0; i < 8; ++i) {
		uint8_t cmd = ADDR_PROM_SETUP + (i * 2);

		_retries = 5;

		if (_readReg(cmd, &prom_buf[0], 2) < 0) {
			DF_LOG_ERR("Read calibration error");
			break;
		}

		// check if all bytes are zero
		if (i == 0) {
			last_val = prom_buf[0];
		}

		if (prom_buf[0] != last_val || prom_buf[1] != last_val) {
			bits_stuck = false;
		}

		cvt.b[0] = prom_buf[1];
		cvt.b[1] = prom_buf[0];
		memcpy(((uint16_t *)&m_sensor_calibration + i), &cvt.w, sizeof(uint16_t));
	}

	DF_LOG_DEBUG("factory_setup: %d", m_sensor_calibration.factory_setup);
	DF_LOG_DEBUG("c1: %d", m_sensor_calibration.c1_pressure_sens);
	DF_LOG_DEBUG("c2: %d", m_sensor_calibration.c2_pressure_offset);
	DF_LOG_DEBUG("c3: %d", m_sensor_calibration.c3_temp_coeff_pres_sens);
	DF_LOG_DEBUG("c4: %d", m_sensor_calibration.c4_temp_coeff_pres_offset);
	DF_LOG_DEBUG("c5: %d", m_sensor_calibration.c5_reference_temp);
	DF_LOG_DEBUG("c6: %d", m_sensor_calibration.c6_temp_coeff_temp);

	return (crc4((uint16_t *)&m_sensor_calibration) && !bits_stuck) ? 0 : -1;
}

int MS5607::ms5607_init()
{
	/* Zero the struct */
	m_synchronize.lock();

	m_sensor_data.pressure_pa = 0.0f;
	m_sensor_data.temperature_c = 0.0f;
	m_sensor_data.last_read_time_usec = 0;
	m_sensor_data.read_counter = 0;
	m_sensor_data.error_counter = 0;
	m_synchronize.unlock();

	int result = _setSlaveConfig(MS5607_SLAVE_ADDRESS,
				     MS5607_BUS_FREQUENCY_IN_KHZ,
				     MS5607_TRANSFER_TIMEOUT_IN_USECS);

	if (result < 0) {
		DF_LOG_ERR("could not set slave config");
	}

	/* Reset sensor and load calibration data into internal register */
	result = reset();

	if (result < 0) {
		DF_LOG_ERR("error: unable to communicate with the MS5607 pressure sensor");
		return -EIO;
	}

	result = loadCalibration();

	if (result != 0) {
		DF_LOG_ERR("error: unable to complete initialization of the MS5607 pressure sensor");
		return -EIO;
	}

	return 0;
}

int MS5607::reset()
{
	int result;
	uint8_t cmd = ADDR_RESET_CMD;
	_retries = 10;
	result = _writeReg(cmd, nullptr, 0);

	if (result < 0) {
		DF_LOG_ERR("Unable to reset device: %d", result);
		return -EIO;
	}

	return result;
}

int MS5607::start()
{
	int result = 0;

	result = I2CDevObj::start();

	if (result != 0) {
		DF_LOG_ERR("error: could not start DevObj");
		goto exit;
	}

	/* Initialize the pressure sensor.*/
	result = ms5607_init();

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

int MS5607::stop()
{
	int result = DevObj::stop();

	if (result != 0) {
		DF_LOG_ERR("DevObj stop failed");
		return result;
	}

	return 0;
}

int MS5607::_request(uint8_t cmd)
{
	int ret;

	_retries = 0;
	ret = _writeReg(cmd, nullptr, 0);

	if (ret < 0) {
		DF_LOG_ERR("error: request failed");
	}

	return ret;
}

int MS5607::_collect(uint32_t *raw)
{
	int ret;

	union {
		uint8_t b[4];
		uint32_t w;
	} cvt;

	uint8_t buf[3];

	_retries = 0;
	uint8_t cmd = ADDR_CMD_ADC_READ;
	ret = _readReg(cmd, &buf[0], 3);

	if (ret < 0) {
		*raw = 0;
		return -1;
	}

	cvt.b[0] = buf[2];
	cvt.b[1] = buf[1];
	cvt.b[2] = buf[0];
	cvt.b[3] = 0;
	*raw = cvt.w;

	return 0;
}

void MS5607::_measure(void)
{

	// Request to convert the temperature
	if (_request(ADDR_CMD_CONVERT_D2) < 0) {
		DF_LOG_ERR("error: temp measure failed");
		return;
	}

	usleep(MS5607_CONVERSION_INTERVAL_US);

	// read the result
	uint32_t temperature_from_sensor;

	if (_collect(&temperature_from_sensor) < 0) {
		DF_LOG_ERR("error: temp collect failed");
		reset();
		return;
	}

	// Request to convert the pressure
	if (_request(ADDR_CMD_CONVERT_D1) < 0) {
		DF_LOG_ERR("error: pressure measure failed");
		return;
	}

	usleep(MS5607_CONVERSION_INTERVAL_US);

	// read the result
	uint32_t pressure_from_sensor;

	if (_collect(&pressure_from_sensor) < 0) {
		DF_LOG_ERR("error: pressure collect failed");
		return;
	}

	m_synchronize.lock();

	m_sensor_data.temperature_c = convertTemperature(temperature_from_sensor) / 100.0;
	m_sensor_data.pressure_pa = convertPressure(pressure_from_sensor);
	m_sensor_data.last_read_time_usec = DriverFramework::offsetTime();
	m_sensor_data.read_counter++;

	_publish(m_sensor_data);

	m_synchronize.signal();
	m_synchronize.unlock();
}

int MS5607::_publish(struct baro_sensor_data &data)
{
	// TBD
	return -1;
}
