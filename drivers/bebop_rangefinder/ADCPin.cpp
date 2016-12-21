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

#include <errno.h>
#include <string.h>

#include "ADCPin.hpp"
#include "DriverFramework.hpp"

using namespace DriverFramework;

ADCPin::ADCPin(uint16_t device_id, uint16_t channel, uint16_t buffer_length)
	: m_dev_id(device_id), m_dev_path{0}, m_channel(channel), m_buffer_length(buffer_length), m_buffer_enabled(false)
{

	// Prepare the device path
	snprintf(m_dev_path, sizeof(m_dev_path), "/sys/bus/iio/devices/iio:device%d", m_dev_id);
	DF_LOG_INFO("Initialize device: %s", m_dev_path);

	// Before we setup the device, disable the pin (prevent resource busy error)
	disable();

	// Enable the channel
	char path[35] = {0};
	snprintf(path, sizeof(path), "scan_elements/in_voltage%d_en", channel);
	write(path, 1);

	// Set buffer length and disable it initially
	write("/buffer/length", buffer_length);
}

ADCPin::~ADCPin()
{
	// Make sure we leave the device disabled
	disable();
}

int ADCPin::write(const char *path, int value)
{
	char filename[sizeof(m_dev_path) + 20] = {0};
	snprintf(filename, sizeof(filename), "%s/%s", m_dev_path, path);

	FILE *pFile = fopen(filename, "w");

	if (pFile == NULL) {
		DF_LOG_ERR("Unable to open file: %s", filename);
		return -1;
	}

	fprintf(pFile, "%d", value);

	if (fclose(pFile) != 0) {
		DF_LOG_ERR("Unable to close file: %s", filename);
		return -1;
	}

	return 0;
}

int ADCPin::enable()
{
	m_buffer_enabled = true;
	return write("buffer/enable", 1);
}

int ADCPin::disable()
{
	m_buffer_enabled = false;
	return write("buffer/enable", 0);
}

int ADCPin::read(uint16_t *buffer, uint16_t len)
{

	if (!m_buffer_enabled) {
		DF_LOG_ERR("Read not possible, buffer was not enabled");
		return -1;
	}

	FILE *pFile;
	char path[20];
	snprintf(path, sizeof(path), "/dev/iio:device%d", m_dev_id);

	pFile = fopen(path, "r");

	if (pFile == NULL) {
		DF_LOG_ERR("%s", strerror(errno));
		DF_LOG_ERR("Unable to open file: %s", path);
		return -1;
	}

	int result = ::fread(buffer, 2, len, pFile);

	if (result < 0) {
		DF_LOG_ERR("Error reading: %s", strerror(errno));
	}

	result = fclose(pFile);

	if (result < 0) {
		DF_LOG_ERR("Unable to close file: %s", strerror(errno));
	}

	return result;
}
