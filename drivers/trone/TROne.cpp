/****************************************************************************
 *
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
#include "TROne.hpp"
#ifdef __DF_QURT
#include "dev_fs_lib_i2c.h"
#endif

using namespace DriverFramework;

int TROne::start()
{
	int result = I2CDevObj::start();

	if (result != 0) {
		DF_LOG_ERR("error: could not start DevObj");
		goto exit;
	}

	/* Configure the I2C bus parameters for the sensor. */
	result = _setSlaveConfig(slave_addr,
				 TRONE_BUS_FREQUENCY_IN_KHZ,
				 TRONE_TRANSFER_TIMEOUT_IN_USECS);

	if (result != 0) {
		DF_LOG_ERR("I2C slave configuration failed");
		goto exit;
	}

	if (result != 0) {
		DF_LOG_ERR("error: could not start DevObj");
		goto exit;
	}

	/* Probe to check if device is available, otherwise give up. */
	result = probe();

	if (result != 0) {
		DF_LOG_ERR("probing not successful");
		goto exit;
	}

	result = DevObj::start();

	if (result != 0) {
		DF_LOG_ERR("DevObj start not successful");
		DevObj::stop();
		goto exit;

	}

exit:
	return result;
}

int TROne::stop()
{
	int result = DevObj::stop();

	if (result != 0) {
		DF_LOG_ERR("DevObj stop failed");
		return result;
	}

	usleep(100000);

	return 0;
}

int TROne::probe(void)
{
	int result = 0;
	uint8_t who_am_i = 0;

	result = _writeReg(TRONE_WHO_AM_I_REG, (uint8_t *)&who_am_i, sizeof(who_am_i));

	if (result) {
		DF_LOG_ERR("Nonzero result when writing to TRONE_WHO_AM_I_REG 0x%x", TRONE_WHO_AM_I_REG);
		return result;
	}

	result = _simple_read((uint8_t *)&who_am_i, sizeof(who_am_i));

	DF_LOG_DEBUG("WHO_AM_I bytes 0x%02x 0x%02x\n",
		     (unsigned)who_am_i,
		     TRONE_WHO_AM_I_REG_VAL);

	if ((unsigned)who_am_i != TRONE_WHO_AM_I_REG_VAL) {
		DF_LOG_ERR("Wrong WHO_AM_I: 0x%x instead of 0x%x", (unsigned)who_am_i, TRONE_WHO_AM_I_REG_VAL);
		return -1;
	}

	if (result) {
		DF_LOG_ERR("Nonzero result when probing");
		// not found on any address
		return -EIO;
	}

	return 0;
}

void TROne::_measure(void)
{
	int result = 0;

	/* read from the sensor */
	uint8_t val[3] = {0, 0, 0};

	uint8_t dummy = 0; // don't care about what _writeReg writes here

	result = _writeReg(TRONE_MEASURE_REG, (uint8_t *)&dummy, sizeof(dummy));

	if (result) {
		//TODO add error counter
		DF_LOG_ERR("Nonzero result when writing to TRONE_MEASURE_REG 0x%x", TRONE_MEASURE_REG);
		return;
	}

	result = _simple_read(val, sizeof(val));

	if (result) {
		//TODO add error counter
		DF_LOG_ERR("Nonzero result when reading");
		return;
	}

	uint16_t distance_mm = (val[0] << 8) | val[1];

	m_sensor_data.dist = float(distance_mm) * 1e-3f;
	_publish(m_sensor_data);
}

void TROne::set_slave_addr(uint8_t slave)
{
	slave_addr = slave;
}

int TROne::_publish(struct range_sensor_data &data)
{
	// TBD
	return -1;
}
