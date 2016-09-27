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

#include <string.h>
#include "DriverFramework.hpp"
#include "HMC5883.hpp"
#ifdef __DF_QURT
#include "dev_fs_lib_i2c.h"
#endif


#define HMC5883_BUS_FREQUENCY_IN_KHZ	(400)
// Found through trial and error, a timeout of 100 us seems to fail.
#define HMC5883_TRANSFER_TIMEOUT_IN_USECS (500)

#define HMC5883_REG_ID_A		(0x0a)
#define HMC5883_REG_ID_B		(0x0b)
#define HMC5883_REG_ID_C		(0x0c)
#define HMC5883_REG_CONFIG_A		(0x00)
#define HMC5883_REG_CONFIG_B		(0x01)
#define HMC5883_REG_MODE		(0x02)
#define HMC5883_REG_DATA_OUT_X_MSB	(0x03)

#define HMC5883_ID_A		('H')
#define HMC5883_ID_B		('4')
#define HMC5883_ID_C		('3')

#define HMC5883_BITS_CONFIG_A_CONTINUOUS_75HZ (0x6 << 2)

#define HMC5883_BITS_CONFIG_B_RANGE_1GA3    (0x01 << 5)

#define HMC5883_BITS_MODE_CONTINUOUS_MODE  (0x00)
#define HMC5883_BITS_MODE_SINGLE_MODE	    (0x01)

using namespace DriverFramework;


int HMC5883::hmc5883_init()
{

	/* Zero the struct */
	m_synchronize.lock();

	m_sensor_data.last_read_time_usec = 0;
	m_sensor_data.read_counter = 0;
	m_sensor_data.error_counter = 0;
	m_synchronize.unlock();

	int result;
	uint8_t sensor_id;

	/* Read the IDs of the HMC5883 sensor to confirm it's presence. */
	result = _readReg(HMC5883_REG_ID_A, &sensor_id, sizeof(sensor_id));

	if (result != 0) {
		DF_LOG_ERR("error: unable to communicate with the hmc5883 mag sensor");
		return -EIO;
	}

	if (sensor_id != HMC5883_ID_A) {
		DF_LOG_ERR("HMC5883 sensor ID_A returned 0x%x instead of 0x%x", sensor_id, HMC5883_ID_A);
		return -1;
	}

	result = _readReg(HMC5883_REG_ID_B, &sensor_id, sizeof(sensor_id));

	if (result != 0) {
		DF_LOG_ERR("error: unable to communicate with the hmc5883 mag sensor");
		return -EIO;
	}

	if (sensor_id != HMC5883_ID_B) {
		DF_LOG_ERR("HMC5883 sensor ID_B returned 0x%x instead of 0x%x", sensor_id, HMC5883_ID_B);
		return -1;
	}

	result = _readReg(HMC5883_REG_ID_C, &sensor_id, sizeof(sensor_id));

	if (result != 0) {
		DF_LOG_ERR("error: unable to communicate with the hmc5883 mag sensor");
		return -EIO;
	}

	if (sensor_id != HMC5883_ID_C) {
		DF_LOG_ERR("HMC5883 sensor ID_C returned 0x%x instead of 0x%x", sensor_id, HMC5883_ID_C);
		return -1;
	}

	uint8_t config_b = HMC5883_BITS_CONFIG_B_RANGE_1GA3;
	result = _writeReg(HMC5883_REG_CONFIG_B, &config_b, sizeof(config_b));

	if (result != 0) {
		DF_LOG_ERR("error: sensor configuration B failed");
		return -EIO;
	}


	usleep(1000);
	return 0;
}

int HMC5883::start()
{
	int result = I2CDevObj::start();

	if (result != 0) {
		DF_LOG_ERR("error: could not start DevObj");
		goto exit;
	}

	/* Configure the I2C bus parameters for the mag sensor. */
	result = _setSlaveConfig(HMC5883_SLAVE_ADDRESS,
				 HMC5883_BUS_FREQUENCY_IN_KHZ,
				 HMC5883_TRANSFER_TIMEOUT_IN_USECS);

	if (result != 0) {
		DF_LOG_ERR("I2C slave configuration failed");
		goto exit;
	}

	/* Initialize the mag sensor. */
	result = hmc5883_init();

	if (result != 0) {
		DF_LOG_ERR("error: mag sensor initialization failed, sensor read thread not started");
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

int HMC5883::stop()
{
	int result = DevObj::stop();

	if (result != 0) {
		DF_LOG_ERR("DevObj stop failed");
		return result;
	}

	usleep(100000);

	return 0;
}

void HMC5883::_measure(void)
{
	int result = 0;

	if (_measurement_requested) {

#pragma pack(push, 1)
		struct { /* status register and data as read back from the device */
			int16_t		x;
			int16_t		z;
			int16_t		y;
		}	hmc_report;
#pragma pack(pop)

		result = _readReg(HMC5883_REG_DATA_OUT_X_MSB, (uint8_t *)&hmc_report, sizeof(hmc_report));

		if (result != 0) {
			m_synchronize.lock();
			m_sensor_data.error_counter++;
			m_synchronize.unlock();

		} else {

			// TODO: add define if byteswap is needed
			hmc_report.x = swap16(hmc_report.x);
			hmc_report.y = swap16(hmc_report.y);
			hmc_report.z = swap16(hmc_report.z);

			m_synchronize.lock();

			m_sensor_data.field_x_ga = hmc_report.x * (1.0f / 1090.0f);
			m_sensor_data.field_y_ga = hmc_report.y * (1.0f / 1090.0f);
			m_sensor_data.field_z_ga = hmc_report.z * (1.0f / 1090.0f);
			m_sensor_data.last_read_time_usec = DriverFramework::offsetTime();
			m_sensor_data.read_counter++;

			_publish(m_sensor_data);

			m_synchronize.signal();
			m_synchronize.unlock();

		}
	}


	/* Request next measurement. */
	uint8_t mode = HMC5883_BITS_MODE_SINGLE_MODE;
	result = _writeReg(HMC5883_REG_MODE, &mode, sizeof(mode));

	if (result != 0) {
		// TODO: count it as an error and keep going
		DF_LOG_ERR("error: setting sensor mode failed");
	}

	_measurement_requested = true;
}

int HMC5883::_publish(struct mag_sensor_data &data)
{
	// TBD
	return -1;
}
