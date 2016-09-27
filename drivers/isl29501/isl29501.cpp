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
#include "isl29501.hpp"
#define _USE_MATH_DEFINES
#include <math.h>
#ifdef __DF_QURT
#include "dev_fs_lib_i2c.h"
#endif
#define REF_DIST 1.0f
using namespace DriverFramework;

//Open connection and initialise ISL distance sensor
int ISL29501::start()
{

	int result = I2CDevObj::start();

	if (result != 0) {
		DF_LOG_ERR("error: could not start DevObj");
		goto exit;
	}

	/* Configure the I2C bus parameters for the sensor. */
	_slave_addr = _detect();

	if (_slave_addr == 0) {
		_detected = true;

	} else {
		DF_LOG_ERR("Unable to connect to the device: %s", m_dev_path);
		goto exit;
	}

	init_params();
exit:

	if (result != 0) {
		DF_LOG_ERR("error: Failed to start ISL");
		I2CDevObj::stop();
	}

	return result;
}

// Configure ISL Sensor
int ISL29501::init_params()
{
	write_reg(0x01, 0x00); //Master Control
	write_reg(0x10, 0x07); //Sample Len
	write_reg(0x11, 0x6E); //Sample Period
	write_reg(0x12, 0x00); //Sample Range
	write_reg(0x13, 0x75); //Sample Range
	write_reg(0x14, 0x02); //DC Cal
	write_reg(0x15, 0x02); //ZP Cal
	write_reg(0x16, 0x02); //Collision
	write_reg(0x1A, 0x0F); //DSP Enable
	write_reg(0x1B, 0x7F); //DC Sat Thres
	write_reg(0x1C, 0x0C); //Noise Thres Offset
	write_reg(0x1D, 0x02); //Noise Thres Slope
	write_reg(0x1E, 0x00); //Mag Sq Exp
	write_reg(0x1F, 0x00); //Mag Sq
	write_reg(0x20, 0x1A); //Circuit Noise
	write_reg(0x21, 0x10); //Noise Gain
	write_reg(0x22, 0x0A); //Fixed Error
	write_reg(0x24, 0x48); //Cross Talk I MSB
	write_reg(0x25, 0x54); //Cross Talk I LSB
	write_reg(0x26, 0x36); //Cross Talk I EXP
	write_reg(0x27, 0x45); //Cross Talk Q MSB
	write_reg(0x28, 0xB0); //Cross Talk Q LSB
	write_reg(0x29, 0xB1); //Cross Talk Q EXP
	write_reg(0x2A, 0xFC); //Cross Talk Gain MSB
	write_reg(0x2B, 0xCA); //Cross Talk Gain LSB
	write_reg(0x2C, 0x07); //Mag EXP
	write_reg(0x2D, 0x9B); //Mag MSB
	write_reg(0x2E, 0xD0); //Mag LSB
	write_reg(0x2F, 0x12); //Phase Offset MSB
	write_reg(0x30, 0x15); //Phase Offset LSB
	write_reg(0x60, 0x01); //Interrupt Control
	write_reg(0x61, 0x00); //Data Invalid Msk
	write_reg(0x62, 0x00); //Detection Control
	write_reg(0x63, 0x00); //Detection Cnd 1
	write_reg(0x64, 0x00); //Detection Cnd 2
	write_reg(0x70, 0x00); //Dist Th 1 Hgh MSB
	write_reg(0x71, 0xDB); //Dist Th 1 Hgh LSB
	write_reg(0x72, 0xFF); //Dist Th 1 Low MSB
	write_reg(0x73, 0x9E); //Dist Th 1 Low LSB
	write_reg(0x74, 0x05); //Dist Th 2 Hgh MSB
	write_reg(0x75, 0x61); //Dist Th 2 Hgh LSB
	write_reg(0x76, 0x00); //Dist Th 2 Low MSB
	write_reg(0x77, 0xB1); //Dist Th 2 Low LSB
	write_reg(0x78, 0x0F); //Dist Th 3 Hgh MSB
	write_reg(0x79, 0x5E); //Dist Th 3 Hgh LSB
	write_reg(0x7A, 0x03); //Dist Th 3 Low MSB
	write_reg(0x7B, 0xD7); //Dist Th 3 Low LSB
	write_reg(0x7D, 0x95); //Mag Th 1 Hgh
	write_reg(0x7E, 0x02); //Mag Th 1 EXP
	write_reg(0x7F, 0xC7); //Mag Th 1 Low
	write_reg(0x81, 0xA3); //Mag Th 2 Hgh
	write_reg(0x82, 0x06); //Mag Th 2 EXP
	write_reg(0x83, 0x94); //Mag Th 2 Low
	write_reg(0x84, 0x02); //Motion Dist Th MSB
	write_reg(0x85, 0x4E); //Motion Dist Th LSB
	write_reg(0x86, 0x00); //Motion MAG Th
	write_reg(0x90, 0x0F); //Driver Range
	write_reg(0x91, 0x80); //Emitter DAC
	write_reg(0x92, 0x00); //Driver Control
	write_reg(0x93, 0x00); //Thresh DAC
	write_reg(0x94, 0x00); //Driver Boost
	write_reg(0x96, 0x00); //Driv Chg Bal DAC
	write_reg(0x97, 0x00); //Front End Ctl
	write_reg(0x98, 0x00); //AFE Ctl Regs
	write_reg(0xA5, 0x00); //Emitter ADC Offset
	write_reg(0xA7, 0x00); //Temp Sens Reg A
	write_reg(0xA8, 0x00); //Temp Sens Reg B
	write_reg(0xA9, 0x00); //Temp Sens Reg C
	write_reg(0xAA, 0x00); //Temp Sens ADC Mode
	write_reg(0xae, 0x00); //Internal Reset
	write_reg(0x31, 0x00); //Temp Reference
	write_reg(0x33, 0x00); //Phase Exponent
	write_reg(0x34, 0x00); //Temp Coeff 1
	write_reg(0x39, 0x00); //Temp Coeff 2
	write_reg(0x36, 0x00); //Ambient Coeff 1
	write_reg(0x3B, 0x00); //Ambient Coeff 2
	unsigned char sample_delay_reg = read_reg(0x10); // Sample Len
	_isl_sample_delay = calc_sample_delay(sample_delay_reg);
	return 0;
}

//Calculate sample period as per value received from Sensor
//calculations based on information provided by intersil
int ISL29501::calc_sample_delay(unsigned char value)
{
	float d = 0.0711 * pow(2, value);
	return (int) d + 1;
}

//Register write function call for I2C bus
int ISL29501::write_reg(uint8_t address, uint8_t data)
{
	return _writeReg(address, &data, sizeof(data));
}

//Register read function call for I2C bus
uint8_t ISL29501::read_reg(uint8_t address)
{
	uint8_t ret = 0;

	if (_readReg(address, &ret, sizeof(ret)) == -1) {
		_read_failure = true;
		DF_LOG_ERR("Read Failure");
	}

	return ret;
}

//Stop ISL distance sensor driver
int ISL29501::stop()
{
	int result = DevObj::stop();

	if (result != 0) {
		DF_LOG_ERR("DevObj stop failed");
		return result;
	}

	usleep(100000);

	return 0;
}

//ISL sensor calibration routine:
//User is required to put sensor at REF_DIST and run this routine to get
//calibration value.
//TODO: Make the calculated offset persistent using px4 param system
int ISL29501::calibration(void)
{
	uint16_t raw_data;
	float raw_avg = 0;
	write_reg(0x2f, 0);	//DIST_CAL_OFF_MSB
	write_reg(0x30, 0);	//DIST_CAL_OFF_LSB
	usleep(100000);

	for (uint8_t i = 0; i < 100; i++) {
		raw_data = read_reg(0xd2);		//DIST_REG_LSB
		raw_data |= (read_reg(0xd1)) << 8; //DIST_REG_MSB
		raw_avg += float(raw_data) / 100;
	}

	raw_avg /= 2000;	//convert to meters
	float offset;
	offset = (raw_avg - REF_DIST) * 2000;

	if (offset < 0) {
		return -1;
	}

	uint8_t off_lsb, off_msb;
	off_lsb = (0xFF & uint16_t(offset));
	off_msb = (0xFF00 & uint16_t(offset)) >> 8;

	write_reg(0x2f, off_msb);
	write_reg(0x30, off_lsb);
	return 0;
}

//Routine to scan through I2C bus for ISL sensor
int ISL29501::_detect()
{
	uint8_t dev_id;
	int result;

	for (uint8_t i = 0; i <= 3; i++) {
		result = _setSlaveConfig(_slave_addr,
					 ISL_BUS_FREQUENCY_IN_KHZ,
					 ISL_TRANSFER_TIMEOUT_IN_USECS);

		if (result != 0) {
			_slave_addr++;
			continue;
		}

		result = DevObj::start();

		if (result == 0) {
			dev_id = read_reg(0x00);

			if (dev_id == 0x0A) {
				_detected = true;
				DF_LOG_INFO("Detected ISL sensor @ 0x%x", _slave_addr);
				return 0;
			}
		}

		_slave_addr++;
		DevObj::stop();
	}

	return -1;
}

// read distance data from ISL sensor
void ISL29501::_measure(void)
{
	if (!_detected) {
		return;
	}

	float distance;
	//setup for reading sensor
	write_reg(0xB0, 0x49);
	uint16_t data_msb, data_lsb, prec;
	uint16_t offset;
	uint8_t inter, data_invalid;

	//read data ready flag
	inter = read_reg(0x69);

	//read sensor offset
	offset = read_reg(0x30);
	offset |= (read_reg(0x2f) << 8);

	//read data validity
	data_invalid = read_reg(0xd0);

	//read corrected data
	data_lsb = read_reg(0xd2);
	data_msb = read_reg(0xd1);

	//check if data was ready
	if (!inter) {
		DF_LOG_ERR("Bad Distance Data Flags: 0x%x 0x%x", data_invalid, inter);
		return;
	}

	//read quality of data
	//TODO: send this data to EKF for processing
	prec = read_reg(0xd4);
	prec |= ((uint16_t)read_reg(0xd3)) << 8;
	//read temperature of sensor
	//TODO: do temperature correction
	uint8_t temperature;
	temperature = read_reg(0xe2);
	//convert distance data to meters
	distance = ((data_msb << 8) + data_lsb) / 2000.0f;
	//correct distance from errors due to phase-distance inconsistency
	float phase = (distance / 33.3f) * M_PI_F * 2.0f;
	distance += (1.1637f * phase * phase * phase - 3.8654f * phase * phase + 1.3796f * phase - 0.0436f);
	DF_LOG_DEBUG("distance: %f offset: 0x%x ", (double)distance, offset);

	if (distance < 0) {
		distance = 0;
	}

	//pack data into structure
	m_sensor_data.temperature = float(temperature);
	m_sensor_data.dist = distance;

	//do not publish data if any of the reads failed
	if (!_read_failure) {
		_publish(m_sensor_data);
	}

	_read_failure = false;
}

//set slave address of the ISL senso
void ISL29501::set_slave_addr(uint8_t slave)
{
	_slave_addr = slave;
}

int ISL29501::_publish(struct range_sensor_data &data)
{
	// declared in wrapper
	return -1;
}
