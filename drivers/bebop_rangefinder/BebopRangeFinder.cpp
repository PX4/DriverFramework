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

#define SPEED_OF_SOUND 343.2f
#define ADC_SAMPLING_FREQ_HZ 160000.0f

// Threshold to detect the beginning or our send pulse
#define BEBOP_RANGEFINDER_REQUEST_THRESH 3276

// The mic records the transmitted signal at the beginning of our received signal.
// This value defines, when it should have ended and is used to detect if we are bellow
// block distance.
#define BEBOP_RANGEFINDER_SEND_PULSE_LEN 420

// This value defines the end of the transitted signal and defines the noise level. A valid
// received signal has to be above this value to be accepted
#define BEBOP_RANGEFINDER_NOISE_LEVEL_THRESH 100

using namespace DriverFramework;

BebopRangeFinder::BebopRangeFinder(const char *device_path) :
	SPIDevObj("BebopRangeFinder", device_path, BEBOP_RANGEFINDER_CLASS_PATH, BEBOP_RANGEFINDER_MEASURE_INTERVAL_US),
	m_sonar_pin(0, 2, BEBOP_RANGEFINDER_BUFFER_LEN), m_requested_data(false),
	m_send_length(0), m_maximum_signal_value(0)
{
	m_id.dev_id_s.devtype = DRV_DF_DEVTYPE_BEBOP_RANGEFINDER;
	m_id.dev_id_s.address = DRV_DF_DEVTYPE_BEBOP_RANGEFINDER;

	// Generate the pulse send via SPI and emitter
	memset(m_pulse, 0x00, BEBOP_RANGEFINDER_PULSE_LEN);
	memset(m_pulse, 0xF0, 16);

	// Initialize the buffers with zero
	memset(m_read_buffer, 0, BEBOP_RANGEFINDER_BUFFER_LEN * 2);
	memset(m_filtered_buffer, 0, BEBOP_RANGEFINDER_BUFFER_LEN * 2);
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
	return result;
}

int16_t BebopRangeFinder::_find_end_of_send()
{
	bool peak = false;
	uint16_t start_index = 0;
	uint16_t end_index = 0;

	for (unsigned int i = 0; i < BEBOP_RANGEFINDER_BUFFER_LEN; ++i) {
		uint16_t value = m_read_buffer[i] >> 4;

		if (!peak && value > BEBOP_RANGEFINDER_REQUEST_THRESH) {
			start_index = i;
			peak = true;

		} else if (peak && value < (BEBOP_RANGEFINDER_NOISE_LEVEL_THRESH)) {
			end_index = i;
			break;
		}
	}

	m_send_length = end_index - start_index;

	if (m_send_length > BEBOP_RANGEFINDER_SEND_PULSE_LEN) {
		DF_LOG_DEBUG("Bellow block distance");
		return -1;

	} else {
		return end_index;
	}
}

int16_t BebopRangeFinder::_filter_read_buffer()
{

	memset(m_filtered_buffer, 0, BEBOP_RANGEFINDER_BUFFER_LEN * 2);
	m_maximum_signal_value = 0;

	int16_t eos = _find_end_of_send(); // get index where the send pulse ends (end-of-send = eos)

	if (eos < 0) {
		return -1;
	}

	// Perform a rolling mean filter with size 3
	// Raw values on the Bebop's iio have the format (16 >> 4)

	// Set the values at the left edge
	m_filtered_buffer[0] = m_read_buffer[eos] >> 4;
	m_filtered_buffer[1] = m_read_buffer[eos + 1] >> 4;

	uint16_t running_sum = m_filtered_buffer[0] + m_filtered_buffer[1]
			       + (m_read_buffer[eos + 2] >> 4);

	// Mean filter the read signal, but exclude the pulse recorded during send
	for (unsigned int i = 2; i < BEBOP_RANGEFINDER_BUFFER_LEN - eos - 1; ++i) {
		m_filtered_buffer[i] = running_sum / 3;
		running_sum = (running_sum + (m_read_buffer[eos + i + 1] >> 4)
			       - (m_read_buffer[eos + i - 2] >> 4));

		// Capture the maximum value in the signal
		if (m_filtered_buffer[i] > m_maximum_signal_value) {
			m_maximum_signal_value = m_filtered_buffer[i];
		}
	}

	if (m_maximum_signal_value < BEBOP_RANGEFINDER_NOISE_LEVEL_THRESH) {
		DF_LOG_DEBUG("No peak found");
		return -1;
	}

	return 0;
}

int16_t BebopRangeFinder::_get_echo_index()
{
	if (_filter_read_buffer() < 0) {
		return -1;
	}

	// threshold is 4/5 * m_maximum_signal_value
	uint16_t threshold = m_maximum_signal_value - (m_maximum_signal_value / 5);
	bool peak = false;
	bool max_peak_found = false;
	uint16_t start_index = 0;

	// search the filtered signal for the rising edge of the peak, which also
	// includes the maximum value (strongest reflection)
	for (unsigned int i = 0; i < BEBOP_RANGEFINDER_BUFFER_LEN; ++i) {
		if (!peak && m_filtered_buffer[i] > (threshold)) {
			peak = true;
			start_index = i;

		} else if (peak && m_filtered_buffer[i] < threshold) {
			peak = false;

			if (max_peak_found) {
				// return the index of the rising edge and add the length that we cut
				// at the beginning of the signal to exclude the send peak
				return start_index + m_send_length;
			}

		} else if (peak && m_filtered_buffer[i] >= m_maximum_signal_value) {
			max_peak_found = true;
		}
	}

	DF_LOG_DEBUG("No peak");
	return -1;
}

void BebopRangeFinder::_measure()
{
	// Make sure, we requested some data
	if (m_requested_data) {
		if (_collect() >= 0) {

			// Get the index of the reflected pulse and compute the distance
			int16_t echo = _get_echo_index();
			float height_m = 0.0f;

			if (echo >= 0) {
				float index = static_cast<float>(echo);
				height_m = (index * SPEED_OF_SOUND) / (2.0f * ADC_SAMPLING_FREQ_HZ);

			} else {
				height_m = -1.0f;
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

int BebopRangeFinder::_publish(struct bebop_range &data)
{
	// TBD
	return -1;
}
