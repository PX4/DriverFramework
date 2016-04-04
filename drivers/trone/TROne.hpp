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

#pragma once

#include "I2CDevObj.hpp"

/* Configuration Constants */

#define TRONE_DEVICE_PATH   "/dev/iic-9"  // J15
#define TRONE_CLASS_PATH  	"/dev/trone"

/* TRONE Registers addresses */
#define TRONE_MEASURE_REG	0x00		/* Measure range register */
#define TRONE_WHO_AM_I_REG  0x01        /* Who am I test register */
#define TRONE_WHO_AM_I_REG_VAL 0xA1

/* Device limits in m*/
#define TRONE_MIN_DISTANCE (0.20f)
#define TRONE_MAX_DISTANCE (14.00f)

#define TRONE_CONVERSION_INTERVAL 50000 /* 50ms */

// TODO: no idea what to use here
#define TRONE_MEASURE_INTERVAL_US (1000000/150)

#define TRONE_SLAVE_ADDRESS		(0x30)

// up to 100kHz
#define TRONE_BUS_FREQUENCY_IN_KHZ	(100)
// TODO: no idea what to use here
#define TRONE_TRANSFER_TIMEOUT_IN_USECS (9000)

namespace DriverFramework
{
struct range_sensor_data {
	float dist; // the distance in m
};

class TROne : public I2CDevObj
{
public:
	TROne(const char *device_path) :
		I2CDevObj("RangeFinder", device_path, TRONE_CLASS_PATH, TRONE_MEASURE_INTERVAL_US)
	{
		set_slave_addr(TRONE_SLAVE_ADDRESS);
	}

	// @return 0 on success, -errno on failure
	virtual int start();

	// @return 0 on success, -errno on failure
	virtual int stop();

	// @return 0 on success
	int probe();

	void set_slave_addr(uint8_t slave);

protected:
	virtual void _measure();
	virtual int _publish(struct range_sensor_data &data);

	struct range_sensor_data		m_sensor_data;

private:
	// returns 0 on success, -errno on failure
	int trone_init();

	int slave_addr;
};

}; // namespace DriverFramework
