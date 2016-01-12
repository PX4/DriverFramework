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
#define __STDC_FORMAT_MACROS
#include <inttypes.h>
#include <stdio.h>
#include <unistd.h>
#include <unistd.h>
#include "DriverFramework.hpp"
#include "MPU9050.hpp"

using namespace DriverFramework;

class ImuTester
{
public:
	static const int TEST_PASS = 0;
	static const int TEST_FAIL = 1;

	ImuTester() :
		m_sensor(IMU_DEVICE_PATH)
	{}

	static void readSensorCallback(void *arg);

	int run(void);

private:
	void readSensor();
	void wait();

	MPU9050		m_sensor;
	uint32_t 	m_read_attempts = 0;
	uint32_t 	m_read_counter = 0;

	int		m_pass;
	bool		m_done = false;
};

static void printImuValues()
{
	DF_LOG_INFO("Hello MPU9050");
}

int ImuTester::run()
{
	DF_LOG_INFO("Entering: run");
	// Default is fail unless pass critera met
	m_pass = TEST_FAIL;

	// Register the driver
	int ret = m_sensor.init();

	// Open the IMU sensor
	DevHandle h;
	DevMgr::getHandle(IMU_DEVICE_PATH, h);
	if (!h.isValid())
	{
		DF_LOG_INFO("Error: unable to obtain a valid handle for the receiver at: %s (%d)",
			IMU_DEVICE_PATH, h.getError());
		m_done = true;
	}
	else {
		m_done = false;
	}

	//while (!m_done) {
		//++m_read_attempts;
		//ret = ImuSensor::getSensorData(h, true);
		//if (ret == 0) {
		//	uint32_t count = m_sensor_data.sensor_read_counter;
		//	if (m_read_counter != count) {
		//		m_read_counter = count;
		//		printImuValues(m_sensor_data);
		//	}
		//}
		//else {
		//	DF_LOG_INFO("error: unable to read the IMU sensor device.");
		//}

		//if ((m_read_counter >= 1000) && (m_read_attempts == m_read_counter)) {
		//	// Done test - PASSED
		//	m_pass = TEST_PASS;
		//	m_done = true;
		//}
		//else if (m_read_attempts > 1000) {
		//	DF_LOG_INFO("error: unable to read the IMU sensor device.");
		//	m_done = true;
		//}
	//}

	DF_LOG_INFO("Closing IMU sensor\n");
	m_sensor.stop();
	return m_pass;
}

int do_test()
{
	int ret = Framework::initialize();
	if (ret < 0) {
		return ret;
	}

	ImuTester pt;

	DF_LOG_INFO("Run it");
	ret = pt.run();

	Framework::shutdown();

	DF_LOG_INFO("Test %s", (ret == ImuTester::TEST_PASS) ? "PASSED" : "FAILED");
	return ret;
}

