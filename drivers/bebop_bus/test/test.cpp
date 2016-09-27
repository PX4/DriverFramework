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
#include <unistd.h>
#include "DriverFramework.hpp"
#include "BebopBus.hpp"


using namespace DriverFramework;

class BebopBusTester : public BebopBus
{
public:
	static const int TEST_PASS = 0;
	static const int TEST_FAIL = 1;

	BebopBusTester() :
		BebopBus(BEBOP_BUS_DEVICE_PATH)
	{}

	int run(void);

private:

	int		m_pass;
	bool	m_done = false;

	int print_observation();
	int test_gpios();
	int test_sound();
	int test_motors();
};

int BebopBusTester::print_observation()
{
	struct bebop_bus_observation data = {0};

	if (_get_observation_data(&data) != 0) {
		return -1;
	}

	DF_LOG_INFO("Observation data:");
	DF_LOG_INFO("  RPM Front-Left: %d", data.rpm_front_left);
	DF_LOG_INFO("      Front-Right: %d", data.rpm_front_right);
	DF_LOG_INFO("      Back-Right: %d", data.rpm_back_right);
	DF_LOG_INFO("      Back-Left: %d", data.rpm_back_left);
	DF_LOG_INFO("  Battery Voltage: %d", data.battery_voltage_mv);
	DF_LOG_INFO("  Status: %s", strstatus(data.status));
	DF_LOG_INFO("  Motors in fault: %X", data.motors_in_fault);
	DF_LOG_INFO("  Errno: %d", data.error);
	DF_LOG_INFO("  Temperature: %d \n", data.temperatur_c);

	return 0;
}

int BebopBusTester::run()
{
	DF_LOG_INFO("Entering: run");
	// Default is fail unless pass critera met
	m_pass = TEST_PASS;

	// Register the driver
	int ret = init();

	if (ret) {
		DF_LOG_ERR("Driver init failed");
		return ret;
	}

	// Open the mag sensor
	DevHandle h;
	DevMgr::getHandle(BEBOP_BUS_DEVICE_PATH, h);

	if (!h.isValid()) {
		DF_LOG_ERR("Unable to obtain a valid handle for the receiver at: %s (%d)",
			   BEBOP_BUS_DEVICE_PATH, h.getError());
		m_done = true;

	} else {
		m_done = false;
	}

	m_pass += test_gpios();
	usleep(1000000);
	m_pass += test_sound();
	usleep(1000000);
	m_pass += test_motors();
	usleep(1000000);

	DF_LOG_INFO("Closing Bebop bus\n");
	stop();
	return m_pass > 0;
}

int BebopBusTester::test_motors()
{
	int pass = TEST_PASS;

	if (print_observation() != 0) {
		pass += TEST_FAIL;
	}

	DF_LOG_INFO("Start Motors");

	if (_start_motors() != 0) {
		DF_LOG_ERR("Start Motors: Failed");
		pass += TEST_FAIL;
	}

	if (print_observation() != 0) {
		pass += TEST_FAIL;
	}

	usleep(2000000);
	float test_speeds[] = {0.0, 0.25, 0.5, 1.0, 0.5, 0.0};
	float speeds[4] = {0};

	for (size_t i = 0; i < (sizeof(test_speeds) / sizeof(float)); ++i) {
		for (size_t j = 0; j < (sizeof(speeds) / sizeof(float)); ++j) {
			speeds[j] = test_speeds[i];
		}

		_set_esc_speed(speeds);
		usleep(500000);

		if (print_observation() != 0) {
			pass += TEST_FAIL;
		}

		usleep(50000);
	}

	if (print_observation() != 0) {
		pass += TEST_FAIL;
	}

	DF_LOG_INFO("Stop Motors");

	if (_stop_motors() != 0) {
		DF_LOG_ERR("Stop Motors: Failed");
		pass += TEST_FAIL;
	}

	if (print_observation() != 0) {
		pass += TEST_FAIL;
	}

	usleep(2500000);

	if (print_observation() != 0) {
		pass += TEST_FAIL;
	}

	if (pass == 0) {
		DF_LOG_INFO("Motor Test: Passed\n");

	} else {
		DF_LOG_INFO("Motor Test: Failed\n");
	}

	return pass > 0;
}

int BebopBusTester::test_gpios()
{
	int pass = TEST_PASS;

	DF_LOG_INFO("Red LEDs");

	if (_toggle_gpio(RED) != 0) {
		DF_LOG_ERR("Failed");
		pass += TEST_FAIL;
	}

	usleep(2000000);

	DF_LOG_INFO("Green LEDs");

	if (_toggle_gpio(GREEN) != 0) {
		DF_LOG_ERR("Failed");
		pass += TEST_FAIL;
	}

	usleep(2000000);

	DF_LOG_INFO("Reset LEDs");

	if (_toggle_gpio(RESET) != 0) {
		DF_LOG_ERR("Failed");
		pass += TEST_FAIL;
	}

	usleep(2000000);

	if (pass == 0) {
		DF_LOG_INFO("GPIO Test: Passed\n");

	} else {
		DF_LOG_INFO("GPIO Test: Failed\n");
	}

	return pass > 0;
}

int BebopBusTester::test_sound()
{
	int pass = TEST_PASS;

	DF_LOG_INFO("Play SHORT sound");

	if (_play_sound(SHORT) != 0) {
		DF_LOG_ERR("Failed");
		pass += TEST_FAIL;
	}

	usleep(2000000);

	DF_LOG_INFO("Play Boot sound");

	if (_play_sound(BOOT) != 0) {
		DF_LOG_ERR("Failed");
		pass += TEST_FAIL;
	}

	usleep(2000000);

	DF_LOG_INFO("Play Be-Bop-Ah-Lula sound");

	if (_play_sound(MELODY) != 0) {
		DF_LOG_ERR("Failed");
		pass += TEST_FAIL;
	}

	usleep(2000000);

	if (pass == 0) {
		DF_LOG_INFO("Sound Test: Passed\n");

	} else {
		DF_LOG_INFO("Sound Test: Failed\n");
	}

	return pass > 0;
}

int do_test()
{
	int ret = Framework::initialize();

	if (ret < 0) {
		return ret;
	}

	BebopBusTester pt;

	ret = pt.run();

	Framework::shutdown();

	DF_LOG_INFO("Test %s", (ret == BebopBusTester::TEST_PASS) ? "PASSED" : "FAILED");
	return ret;
}

