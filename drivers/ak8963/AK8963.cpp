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


#define AK8963_BUS_FREQUENCY_IN_KHZ 400

#define AK8963_TRANSFER_TIMEOUT_IN_USECS 500

#define AK8963_DEV_ID 0x48

#define AK8963_REG_WIA 0x00
#define AK8963_REG_CNTL1 0x0A
#define AK8963_REG_CNTL2 0x0B
#define AK8963_REG_ASTC 0x0C
#define AK8963_REG_ASAX 0x10
#define AK8963_REG_ASAY 0x11
#define AK8963_REG_ASAZ 0x12
#define AK8963_REG_ST1 0x02
#define AK8963_REG_ST2 0x09
#define AK8963_REG_DATA_X_MSB 0x03

#define AK8963_BITS_CNTL1_MODE_POWER_DOWN 0x00
#define AK8963_BITS_CNTL1_MODE_SINGLE 0x01
#define AK8963_BITS_CNTL1_MODE_CONTINOUS1 0x02
#define AK8963_BITS_CNTL1_MODE_EXTERNAL 0x04
#define AK8963_BITS_CNTL1_MODE_CONTINOUS2 0x06
#define AK8963_BITS_CNTL1_MODE_SELF_TEST 0x08
#define AK8963_BITS_CNTL1_MODE_ROM_ACCESS 0x0F
#define AK8963_BITS_CNTL1_OUTPUT_14BIT 0x00
#define AK8963_BITS_CNTL1_OUTPUT_16BIT 0x10
#define AK8963_BITS_CNTL2_SOFT_RESET 0x01

#define AK8963_BITS_ASTC_NORMAL 0x00
#define AK8963_BITS_ASTC_SELF_TEST 0x40

#define AK8963_BITS_ST1_DRDY 0x01
#define AK8963_BITS_ST1_DOR 0x02

#define AK8963_BITS_ST2_HOFL 0x08
#define AK8963_BITS_ST2_BITM 0x10

// 16bit mode: 0.15uTesla/LSB, 100 uTesla == 1 Gauss
#define MAG_RAW_TO_GAUSS 	 (0.15f / 100.0f)

using namespace DriverFramework;


int AK8963::detect()
{
	uint8_t b = 0;

	// get mag version ID
	int retVal = _readReg(AK8963_REG_WIA, &b, 1);

	if (retVal != 0) {
		DF_LOG_ERR("error reading mag whoami reg: %d", retVal);
		return -1;
	}

	if (b != AK8963_DEV_ID) {
		DF_LOG_ERR("wrong mag ID %u (expected %u)", b, AK8963_DEV_ID);
		return -1;
	}

	return 0;
}

int AK8963::get_sensitivity_adjustment()
{
	// First set power-down mode
	uint8_t bits = AK8963_BITS_CNTL1_MODE_POWER_DOWN;

	if (_writeReg(AK8963_REG_CNTL1, &bits, 1) != 0) {
		DF_LOG_ERR("Sensitivity: Set power down mode");
		return -1;
	}

	usleep(10000);

	// Enable FUSE ROM, since the sensitivity adjustment data is stored in
	// compass registers 0x10, 0x11 and 0x12 which is only accessible in Fuse
	// access mode.
	bits = AK8963_BITS_CNTL1_MODE_ROM_ACCESS;

	if (_writeReg(AK8963_REG_CNTL1, &bits, 1) != 0) {
		DF_LOG_ERR("Sensitivity: Set ROM access mode");
		return -1;
	}

	usleep(10000);

	// Get compass calibration register 0x10, 0x11, 0x12
	// store into context.
	uint8_t asa[3] = {0};

	if (_readReg(AK8963_REG_ASAX, asa, sizeof(asa)) != 0) {
		return -1;
	}

	for (int i = 0; i < 3; ++i) {

		float value = asa[i];
		// H_adj = H * ((ASA-128)*0.5/128 + 1)
		//       = H * ((ASA-128) / 256 + 1)
		// H is the raw compass reading.
		_mag_sens_adj[i] = (value - 128.0f) / 256.0f + 1.0f;
	}

	// Leave in a power-down mode
	bits = AK8963_BITS_CNTL1_MODE_POWER_DOWN;

	if (_writeReg(AK8963_REG_CNTL1, &bits, 1) != 0) {
		DF_LOG_ERR("Sensitivity: Set power down mode");
		return -1;
	}

	usleep(10000);

	return 0;
}

bool AK8963::in_range(float value, float min, float max)
{
	return (min <= value) && (value <= max);
}

int AK8963::run_self_test()
{

	uint8_t bits = AK8963_BITS_CNTL1_MODE_POWER_DOWN;

	if (_writeReg(AK8963_REG_CNTL1, &bits, 1) != 0) {
		DF_LOG_ERR("Test: Set power down mode");
		return -1;
	}

	usleep(1000);
	bits = AK8963_BITS_ASTC_SELF_TEST;

	if (_writeReg(AK8963_REG_ASTC, &bits, 1) != 0) {
		DF_LOG_ERR("Test: Set self test");
		return -1;
	}

	usleep(1000);
	bits = AK8963_BITS_CNTL1_MODE_SELF_TEST | AK8963_BITS_CNTL1_OUTPUT_16BIT;

	if (_writeReg(AK8963_REG_CNTL1, &bits, 1) != 0) {
		DF_LOG_ERR("Sensitivity: Set self test mode");
		return -1;
	}

	usleep(1000);

#pragma pack(push, 1)
	struct { /* status register and data as read back from the device */
		int16_t		x;
		int16_t		y;
		int16_t		z;
	}	ak8963_report;
#pragma pack(pop)

	bool ready = false;

	while (!ready) {
		int result = _readReg(AK8963_REG_ST1, &bits , 1);

		if (result != 0) {
			DF_LOG_ERR("Error reading status");
			return -1;
		}

		if (bits & AK8963_BITS_ST1_DRDY) {
			ready = true;
		}

		usleep(100);
	}

	int result = _readReg(AK8963_REG_DATA_X_MSB, (uint8_t *)&ak8963_report, sizeof(ak8963_report));

	// Check if the measurements are in reasonable range (see the data sheet)
	int passed_test = -1;

	if (in_range(ak8963_report.x * _mag_sens_adj[0], -200.0, 200.0)
	    && in_range(ak8963_report.y * _mag_sens_adj[1], -200.0, 200.0)
	    && in_range(ak8963_report.z * _mag_sens_adj[2], -3200.0, -800.0)) {
		DF_LOG_INFO("Selftest passed!");
		passed_test = 0;

	} else {
		DF_LOG_ERR("Selftest failed!");
		passed_test = -1;
	}

	if (result != 0) {
		DF_LOG_ERR("Error reading data");
		return -1;
	}

	bits = AK8963_BITS_ASTC_NORMAL;

	if (_writeReg(AK8963_REG_ASTC, &bits, 1) != 0) {
		DF_LOG_ERR("Test: Set self test");
		return -1;
	}

	usleep(1000);
	bits = AK8963_BITS_CNTL1_MODE_POWER_DOWN;

	if (_writeReg(AK8963_REG_CNTL1, &bits, 1) != 0) {
		DF_LOG_ERR("Test: Set power down mode");
		return -1;
	}

	usleep(1000);
	return passed_test;
}

int AK8963::ak8963_init()
{

	/* Zero the struct */
	m_synchronize.lock();

	m_sensor_data.last_read_time_usec = 0;
	m_sensor_data.read_counter = 0;
	m_sensor_data.error_counter = 0;
	m_synchronize.unlock();

	// Perform soft-reset
	uint8_t bits = AK8963_BITS_CNTL2_SOFT_RESET;;
	int result = _writeReg(AK8963_REG_CNTL2, &bits, 1);

	if (result < 0) {
		DF_LOG_ERR("AK8963 soft reset failed.");
		return -1;
	}

	usleep(1000);

	// Detect mag presence by reading whoami register
	if (detect() != 0) {
		DF_LOG_ERR("AK8963 mag not detected.");
		return -1;
	}

	// Get mag calibraion data from Fuse ROM
	if (get_sensitivity_adjustment() != 0) {
		DF_LOG_ERR("Unable to read mag sensitivity adjustment");
		return -1;
	}

	// Power on and configure the mag to produce 16 bit data in continuous measurement mode.
	bits = AK8963_BITS_CNTL1_OUTPUT_16BIT | AK8963_BITS_CNTL1_MODE_CONTINOUS2;
	result = _writeReg(AK8963_REG_CNTL1, &bits, 1);

	if (result != 0) {
		DF_LOG_ERR("Unable to configure the magnetometer mode.");
	}

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
	// Leave in a power-down mode
	uint8_t bits = AK8963_BITS_CNTL1_MODE_POWER_DOWN;

	if (_writeReg(AK8963_REG_CNTL1, &bits, 1) != 0) {
		DF_LOG_ERR("Sensitivity: Set power down mode");
		return -1;
	}

	usleep(10000);

	int result = DevObj::stop();

	if (result != 0) {
		DF_LOG_ERR("DevObj stop failed");
		return result;
	}

	return 0;
}

void AK8963::_measure(void)
{
#pragma pack(push, 1)
	struct sample {
		int16_t val[3];
		uint8_t st2;
	};
#pragma pack(pop)
	struct sample ak8963_report;

	uint8_t bits = 0;
	int result = _readReg(AK8963_REG_ST1, &bits , 1);

	if (result != 0) {
		DF_LOG_ERR("Error reading status");
		m_synchronize.lock();
		m_sensor_data.error_counter++;
		m_synchronize.unlock();
		return;
	}

	if (bits & AK8963_BITS_ST1_DRDY) {

		if (bits & AK8963_BITS_ST1_DOR) {
			DF_LOG_INFO("Skipped data");
		}

		result = _readReg(AK8963_REG_DATA_X_MSB, (uint8_t *)&ak8963_report, sizeof(ak8963_report));

		if (result != 0) {
			DF_LOG_ERR("Error reading data");
			m_synchronize.lock();
			m_sensor_data.error_counter++;
			m_synchronize.unlock();
			return;
		}

		if (ak8963_report.st2 & AK8963_BITS_ST2_HOFL) {
			DF_LOG_ERR("Magnetic sensor overflow");
			m_synchronize.lock();
			m_sensor_data.error_counter++;
			m_synchronize.unlock();
			return;
		}

		m_synchronize.lock();

		m_sensor_data.field_x_ga = static_cast<float>(ak8963_report.val[0]) * _mag_sens_adj[0] * MAG_RAW_TO_GAUSS;
		m_sensor_data.field_y_ga = static_cast<float>(ak8963_report.val[1]) * _mag_sens_adj[1] * MAG_RAW_TO_GAUSS;
		m_sensor_data.field_z_ga = static_cast<float>(ak8963_report.val[2]) * _mag_sens_adj[2] * MAG_RAW_TO_GAUSS;
		m_sensor_data.last_read_time_usec = DriverFramework::offsetTime();
		m_sensor_data.read_counter++;

		_publish(m_sensor_data);

		m_synchronize.signal();
		m_synchronize.unlock();

	} else {
		return;
	}

	return;
}

int AK8963::_publish(struct mag_sensor_data &data)
{
	//TBD
	return -1;
}
