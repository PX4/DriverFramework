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
#include "AK8963.hpp"


#define AK8963_BUS_FREQUENCY_IN_KHZ 400 // TODO check setting

#define AK8963_TRANSFER_TIMEOUT_IN_USECS 100 // TODO check setting

using namespace DriverFramework;


int AK8963::ak8963_init()
{

	/* Zero the struct */
	m_synchronize.lock();

	m_sensor_data.last_read_time_usec = 0;
	m_sensor_data.read_counter = 0;
	m_sensor_data.error_counter = 0;
	m_synchronize.unlock();

	// TODO Add implementation

	usleep(1000);
	return 0;
}

int AK8963::start()
{
	int result = I2CDevObj::start();

	if (result != 0) {
		DF_LOG_ERR("error: could not start DevObj");
		goto exit;
	}

	/* Configure the I2C bus parameters for the mag sensor. */
	result = _setSlaveConfig(AK8963_SLAVE_ADDRESS,
				 AK8963_BUS_FREQUENCY_IN_KHZ,
				 AK8963_TRANSFER_TIMEOUT_IN_USECS);

	if (result != 0) {
		DF_LOG_ERR("I2C slave configuration failed");
		goto exit;
	}

	/* Initialize the mag sensor. */
	result = ak8963_init();

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

int AK8963::stop()
{
	int result = DevObj::stop();

	if (result != 0) {
		DF_LOG_ERR("DevObj stop failed");
		return result;
	}

	usleep(100000);

	return 0;
}

void AK8963::_measure(void)
{
	// TODO Add implementation
}

int AK8963::_publish(struct mag_sensor_data &data)
{
	// TBD
	return -1;
}
