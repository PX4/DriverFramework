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

#include "BebopRangeFinder.hpp"
#include "DriverFramework.hpp"

#define BEBOP_RANGEFINDER_CAPTURE_PATH "/home/root/rangefinder.csv"

#define SPEED_OF_SOUND 343.2f
#define ADC_SAMPLING_FREQ_HZ 160000.0f

using namespace DriverFramework;

BebopRangeFinder::BebopRangeFinder(const char *device_path) :
	SPIDevObj("BebopRangeFinder", device_path, BEBOP_RANGEFINDER_CLASS_PATH, BEBOP_RANGEFINDER_MEASURE_INTERVAL_US),
	m_sonar_pin(0, 2, BEBOP_RANGEFINDER_BUFFER_LEN), m_requested_data(false),
	m_extractor(BEBOP_RANGEFINDER_BUFFER_LEN), m_capture_signal(false)
{
	m_id.dev_id_s.devtype = DRV_DF_DEVTYPE_BEBOP_RANGEFINDER;
	m_id.dev_id_s.address = DRV_DF_DEVTYPE_BEBOP_RANGEFINDER;

	// Generate the pulse send via SPI and emitter
	memset(m_pulse, 0x00, BEBOP_RANGEFINDER_PULSE_LEN);
	memset(m_pulse, 0xF0, 16);

	// Initialize the buffers with zero
	memset(m_read_buffer, 0, BEBOP_RANGEFINDER_BUFFER_LEN * 2);
}

int BebopRangeFinder::start()
{
	/* Open the device path specified in the class initialization. */
	// attempt to open device in start()
	int result = SPIDevObj::start();

	if (result != 0) {
		DF_LOG_ERR("Unable to open the device path: %s", m_dev_path);
		return result;
	}

	result = _bebop_rangefinder_init();

	if (result != 0) {
		DF_LOG_ERR("error: Bebop rangefinder sensor initialization failed, sensor read thread not started");
		return result;
	}

	result = DevObj::start();

	if (result != 0) {
		DF_LOG_ERR("DevObj start failed (%d)", result);
		return result;
	}

	return 0;
}

int BebopRangeFinder::stop()
{
	int result = DevObj::stop();

	if (result != 0) {
		DF_LOG_ERR("DevObj stop failed");
		return result;
	}

	return 0;
}

int BebopRangeFinder::_bebop_rangefinder_init()
{
	/* Zero the struct */
	m_synchronize.lock();
	m_sensor_data.height_m = 0.0;
	m_synchronize.unlock();

	/* Set the bus frequency for register get/set. */
	int result = _setBusFrequency(SPI_FREQUENCY_320KHZ);

	if (result != 0) {
		DF_LOG_ERR("failed setting SPI bus frequency: %d", result);
		return -1;
	}

	return 0;
}

int BebopRangeFinder::_request()
{
	if (m_sonar_pin.enable() >= 0) {
		m_requested_data = true;
		return _writeReg(0x0, m_pulse, BEBOP_RANGEFINDER_PULSE_LEN);
	}

	return -1;
}

int BebopRangeFinder::_collect()
{
	int result = m_sonar_pin.read(m_read_buffer, BEBOP_RANGEFINDER_BUFFER_LEN);

	m_sonar_pin.disable();
	m_requested_data = false;

	for (unsigned int i = 0; i < BEBOP_RANGEFINDER_BUFFER_LEN; ++i) {
		m_read_buffer[i] = m_read_buffer[i] >> 4;
	}

	return result;
}

void BebopRangeFinder::_measure()
{
	// Make sure, we requested some data
	if (m_requested_data) {
		if (_collect() >= 0) {

			// Get the index of the reflected pulse and compute the distance
			int16_t echo = m_extractor.get_echo_index(m_read_buffer);
			float height_m = 0.0f;

			if (echo >= 0) {
				float index = static_cast<float>(echo);
				height_m = (index * SPEED_OF_SOUND) / (2.0f * ADC_SAMPLING_FREQ_HZ);

			} else {
				height_m = -1.0f;
			}


			if (m_capture_signal) {
				_dump_signal(BEBOP_RANGEFINDER_CAPTURE_PATH, echo);
				m_capture_signal = false;
			}

			// Publish the measurements
			m_synchronize.lock();

			m_sensor_data.height_m = height_m;

			_publish(m_sensor_data);

			m_synchronize.signal();
			m_synchronize.unlock();
		}

	}

	// Request a new measurement
	if (_request() < 0) {
		DF_LOG_ERR("Request failed");
	}
}

int BebopRangeFinder::_dump_signal(const char *path, int peak_index)
{

	FILE *pFile = fopen(path, "a");

	if (pFile == nullptr) {
		DF_LOG_ERR("Unable to open file: %s", path);
		return -1;
	}

	fprintf(pFile, "%d", peak_index);

	for (size_t i = 0; i < BEBOP_RANGEFINDER_BUFFER_LEN; ++i) {
		uint16_t value = m_read_buffer[i] >> 4;
		fprintf(pFile, ",%u", value);
	}

	fprintf(pFile, "\n");

	fclose(pFile);
	DF_LOG_INFO("Signal captured: %u", ++m_capture_count);

	return 0;
}

int BebopRangeFinder::_publish(struct bebop_range &data)
{
	// TBD
	return -1;
}
