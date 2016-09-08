
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

namespace DriverFramework
{

struct bebop_range {
  float height_m;
} __attribute__((packed));

#define DRV_DF_DEVTYPE_BEBOP_RANGEFINDER 0x99

// update frequency 20 Hz
#define BEBOP_RANGEFINDER_MEASURE_INTERVAL_US 50000
#define BEBOP_RANGEFINDER_CLASS_PATH "/dev/ranger"
#define BEBOP_RANGEFINDER_DEVICE_PATH "/dev/ranger" // TODO set correct path

class BebopRangeFinder : public SPIDevObj
{
public:
  BebopRangeFinder(const char *device_path) :
		SPIDevObj("BebopRangeFinder", device_path, BEBOP_RANGEFINDER_CLASS_PATH, BEBOP_RANGEFINDER_MEASURE_INTERVAL_US),
		m_measure_phase(0)
  {
		m_id.dev_id_s.devtype = DRV_DF_DEVTYPE_BEBOP_RANGEFINDER;
		m_id.dev_id_s.address = DRV_DF_DEVTYPE_BEBOP_RANGEFINDER;
	}

  ~BebopRangeFinder() = default;

	// @return 0 on success, -errno on failure
	virtual int start();

	// @return 0 on success, -errno on failure
	virtual int stop();

protected:
	void _measure();

	virtual int _publish(struct bebop_range &data);

	struct bebop_range m_sensor_data;
	SyncObj 					m_synchronize;

private:

	// @returns 0 on success, -errno on failure
  int bebop_rangefinder_init();

	int m_measure_phase;

};
} // namespace DriverFramework
