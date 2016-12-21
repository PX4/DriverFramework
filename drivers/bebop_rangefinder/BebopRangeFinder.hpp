
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

#include "SPIDevObj.hpp"
#include "ADCPin.hpp"

namespace DriverFramework
{

struct bebop_range {
	float height_m;
} __attribute__((packed));

#define DRV_DF_DEVTYPE_BEBOP_RANGEFINDER 0x99

// update frequency 16.66 Hz (reading 8192 samples at 160 kHz takes 51200 us)
#define BEBOP_RANGEFINDER_MEASURE_INTERVAL_US 60000
#define BEBOP_RANGEFINDER_CLASS_PATH "/dev/ranger"
#define BEBOP_RANGEFINDER_DEVICE_PATH "/dev/spidev1.0"

#define BEBOP_RANGEFINDER_MIN_DISTANCE_M 0.5
#define BEBOP_RANGEFINDER_MAX_DISTANCE_M 8.5

#define BEBOP_RANGEFINDER_BUFFER_LEN 8192

#define BEBOP_RANGEFINDER_PULSE_LEN 32

class BebopRangeFinder : public SPIDevObj
{
public:
	BebopRangeFinder(const char *device_path);

	~BebopRangeFinder() = default;

	// @return 0 on success, -errno on failure
	int start();

	// @return 0 on success, -errno on failure
	int stop();

protected:
	void _measure();

	virtual int _publish(struct bebop_range &data);

	struct bebop_range m_sensor_data;
	SyncObj 					m_synchronize;

private:

	// @returns 0 on success, -errno on failure
	int _bebop_rangefinder_init();

	int _request();
	int _collect();

	int16_t _find_end_of_send();
	int16_t _filter_read_buffer();
	int16_t _get_echo_index();

	ADCPin m_sonar_pin;
	bool m_requested_data;

	uint8_t m_pulse[BEBOP_RANGEFINDER_PULSE_LEN];
	uint16_t m_read_buffer[BEBOP_RANGEFINDER_BUFFER_LEN];
	uint16_t m_filtered_buffer[BEBOP_RANGEFINDER_BUFFER_LEN];
	uint16_t m_send_length;
	uint16_t m_maximum_signal_value;

};
} // namespace DriverFramework
