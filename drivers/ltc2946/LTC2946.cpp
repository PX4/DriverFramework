/****************************************************************************
 *
 *   Copyright (C) 2016 Bharath Ramaswamy. All rights reserved.
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

/***************************************************
 * modified by Christoph Tobler <christoph@px4.io>
 ***************************************************/

#include <string.h>
#include "DriverFramework.hpp"
#include "LTC2946.hpp"
#ifdef __QURT
#include "dev_fs_lib_i2c.h"
#endif

// APM has two LTC chips, one connected to battery, one connected to 5V supply. Also two different
// current sensing resistors are used to measure the corresponding currents (for APM), and one for
// ESC.

#define LTC2946_BUS_FREQUENCY_IN_KHZ 400
#define LTC2946_TRANSFER_TIMEOUT_IN_USECS 9000

#define LTC2946_BUF_SIZE    32


using namespace DriverFramework;

int LTC2946::configure()
{
	uint8_t CTRLA =
		0b01001000;   //offset calib every 128 conv, so sampling takes about 35ms (for both voltage and current together)
	//uint8_t CTRLA = 0b00001000; //Gnd ref, offset evey conv, volt=sense+, alternate volt and curr. if use this setting, it takes about 100ms to sample
	uint8_t CTRLB = 0b00000100;

	//resolution for voltage sensing is 25mV/LSB (not very high... but voltage range is high)

	if (_writeReg(LTC2946_CTRLA_REG, &CTRLA, sizeof(CTRLA))) { return -3; }

	if (_writeReg(LTC2946_CTRLB_REG, &CTRLB, sizeof(CTRLB))) { return -4; }

	return 0;
}


int LTC2946::ltc2946_init()
{
	/* Zero the struct */
	m_synchronize.lock();

	m_sensor_data.board_voltage_V = 0.0f;
	m_sensor_data.board_current_A = 0.0f;

	m_sensor_data.battery_voltage_V = 0.0f;
	m_sensor_data.battery_current_A = 0.0f;

	m_sensor_data.read_counter = 0;
	m_sensor_data.error_counter = 0;

	m_synchronize.unlock();

	if (configure() != 0) {
		DF_LOG_ERR("ltc2946: configure failed!");
		return -1;
	}

	usleep(1000);
	return 0;
}


int LTC2946::start()
{
	int result;

	result = I2CDevObj::start();

	if (result != 0) {
		DF_LOG_ERR("error:LTC2946 Unable to open the device path: %s", m_dev_path);
		goto start_exit;
	}

	DF_LOG_ERR("LTC2946: Open the device path: %s", m_dev_path);

	/* Configure the I2C bus parameters for the LTC2946 sensor. */
	result = _setSlaveConfig(LTC2946_I2C_ADDRESS, LTC2946_BUS_FREQUENCY_IN_KHZ, LTC2946_TRANSFER_TIMEOUT_IN_USECS);

	if (result != 0) {
		DF_LOG_ERR("I2C slave configuration failed");
		goto start_exit;
	}

	/* Initialize the sensor for active and continuous operation. */
	result = ltc2946_init();

	if (result != 0) {
		DF_LOG_ERR("error: sensor initialization failed, sensor read thread not started");
		goto start_exit;
	}

	result = DevObj::start();

	if (result != 0) {
		DF_LOG_ERR("error: could not start DevObj");
		goto start_exit;
	}

start_exit:

	if (result != 0) {
		DF_LOG_ERR("error: Failed to start ISL");
		I2CDevObj::stop();
	}

	return result;
}


int LTC2946::stop()
{
	int result = DevObj::stop();

	if (result != 0) {
		DF_LOG_ERR("DevObj stop failed");
		return result;
	}

	return 0;
}


void LTC2946::_measure(void)
{
	/* Read the data from the LTC2946 sensor. */

	/**************************************
	 * Read 5V supply voltage and current
	 **************************************/
	m_id.dev_id_s.address = LTC2946_I2C_ADDRESS_5V;

	/* Configure the I2C bus parameters for the APM sensor every time */
	if (_setSlaveConfig(LTC2946_I2C_ADDRESS_5V,
			    LTC2946_BUS_FREQUENCY_IN_KHZ,
			    LTC2946_TRANSFER_TIMEOUT_IN_USECS)) {
		DF_LOG_ERR("LTC2946: I2C board slave configuration failed");
		return;
	}

	m_sensor_data.read_counter++;

	// Read raw voltage measurement from 0x1E register (2 bytes)
	uint8_t vraw[2];
	uint8_t iraw[2];

	int volt_read_ret = _readReg(LTC2946_VIN_MSB_REG, vraw, sizeof(vraw));         // 0x1E
	int curr_read_ret = _readReg(LTC2946_DELTA_SENSE_MSB_REG, iraw, sizeof(iraw)); // 0x14

	if ((volt_read_ret == 0) && (curr_read_ret == 0)) {

		uint16_t volt16 = (((uint16_t)vraw[0]) << 8) | vraw[1];  //MSB first
		volt16        >>= 4;                                     //data is 12 bit and left-aligned
		float v_now      = volt16 / 4095.0f * 102.4f;             //102.4V is maximum voltage on this input

		uint16_t curr16 = (((uint16_t)iraw[0]) << 8) | iraw[1];  //MSB first
		curr16        >>= 4;                                     //data is 12 bit and left-aligned

		// float r_sense    = 0.001f;                            //current sense resistor value on Eagle ESC
		// float r_sense    = 0.0005f;                           //current sense resistor value on Eagle APM (main current sense)
		float r_sense    = 0.005f;                               //current sense resistor value on Eagle APM (5V current sense)
		float i_now      = curr16 / 4095.0f * 0.1024f / r_sense;  //0.1024V is maximum voltage on this input, 0.001 Ohm resistor

		// save data
		m_sensor_data.board_voltage_V = v_now;
		m_sensor_data.board_current_A = i_now;

	} else {

		DF_LOG_INFO("LTC2946 unable to read register");
		m_synchronize.lock();
		m_sensor_data.error_counter++;
		m_synchronize.unlock();
		return;
	}

	/**************************************
	 * Read battery supply voltage and current
	 **************************************/
	m_id.dev_id_s.address = LTC2946_I2C_ADDRESS_VBATT;

	/* Configure the I2C bus parameters for the APM sensor every time */
	if (_setSlaveConfig(LTC2946_I2C_ADDRESS_VBATT,
			    LTC2946_BUS_FREQUENCY_IN_KHZ,
			    LTC2946_TRANSFER_TIMEOUT_IN_USECS)) {
		DF_LOG_ERR("LTC2946: I2C battery slave configuration failed");
		return;
	}

	volt_read_ret = _readReg(LTC2946_VIN_MSB_REG, vraw, sizeof(vraw));         // 0x1E
	curr_read_ret = _readReg(LTC2946_DELTA_SENSE_MSB_REG, iraw, sizeof(iraw)); // 0x14

	if ((volt_read_ret == 0) && (curr_read_ret == 0)) {

		uint16_t volt16 = (((uint16_t)vraw[0]) << 8) | vraw[1];  //MSB first
		volt16        >>= 4;                                     //data is 12 bit and left-aligned
		float v_now      = volt16 / 4095.0f * 102.4f;             //102.4V is maximum voltage on this input

		uint16_t curr16 = (((uint16_t)iraw[0]) << 8) | iraw[1];  //MSB first
		curr16        >>= 4;                                     //data is 12 bit and left-aligned

		// float r_sense    = 0.001f;                            //current sense resistor value on Eagle ESC
		float r_sense    =
			0.0005f;                              //current sense resistor value on Eagle APM (main current sense)
		// float r_sense    = 0.005f;                            //current sense resistor value on Eagle APM (5V current sense)
		float i_now      = curr16 / 4095.0f * 0.1024f / r_sense;  //0.1024V is maximum voltage on this input, 0.001 Ohm resistor

		// save data
		m_sensor_data.battery_voltage_V = v_now;
		m_sensor_data.battery_current_A = i_now;

	} else {

		DF_LOG_INFO("LTC2946 unable to read register");
		m_synchronize.lock();
		m_sensor_data.error_counter++;
		m_synchronize.unlock();

		vraw[0] = 0x00;
		vraw[1] = 0x00;
		iraw[0] = 0x00;
		iraw[1] = 0x00;
		return;
	}

	m_synchronize.lock();

	_publish(m_sensor_data);

	m_synchronize.signal();
	m_synchronize.unlock();

}


int LTC2946::_publish(const struct ltc2946_sensor_data &data)
{
	// TBD
	return -1;
}
