/****************************************************************************
 *
 *   Copyright (C) 2015 Julian Oes. All rights reserved.
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
#include "MPU9050.hpp"

#define MPU9050_REG_WHOAMI 0x75
#define MPU9050_WHOAMI 0x71

using namespace DriverFramework;


int MPU9050::mpu9050_init() {

	uint8_t sensor_id;
	int result = _readReg(MPU9050_REG_WHOAMI, &sensor_id, sizeof(sensor_id));
	if (result != 0) {
		DF_LOG_ERR("error: unable to communicate with the MPU9050 sensor");
		return -EIO;
	}
	DF_LOG_ERR("MPU9050 sensor whoami: 0x%X, should be: 0x%X", sensor_id, MPU9050_WHOAMI);

	usleep(10000);
	return 0;
}

int MPU9050::start()
{
	int result = devOpen(0);
	/* Open the device path specified in the class initialization */
	if (result < 0) {
		DF_LOG_ERR("Unable to open the device path: %s", m_dev_path);
		result = -1;
		goto exit;
	}

    SPIDevObj::start();

	result = mpu9050_init();
	if (result != 0) {
		DF_LOG_ERR("error: IMU sensor initialization failed, sensor read thread not started");
		goto exit;
	}

exit:
	if (result != 0) {
		devClose();
	}
	else {
		result = SPIDevObj::start();
	}

	return result;
}

void MPU9050::_measure(void)
{
}
