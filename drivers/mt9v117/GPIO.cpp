/****************************************************************************
 *
 *   Copyright (C) 2017 PX4 Development Team. All rights reserved.
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

#include "GPIO.hpp"
#include "DriverFramework.hpp"

using namespace DriverFramework;

GPIO::GPIO(uint16_t device_id) :
	m_dev_id(device_id)
{
	// Prepare the device path
	snprintf(m_dev_path, sizeof(m_dev_path), "/sys/class/gpio/gpio%d", m_dev_id);
	DF_LOG_INFO("Initialize device: %s", m_dev_path);

	set_value(false);
}

GPIO::~GPIO()
{
	set_value(false);
}

int GPIO::set_value(bool value)
{
	return write("value", value);
}

int GPIO::write(const char *path, int value)
{
	char filename[sizeof(m_dev_path) + 20] = {0};
	snprintf(filename, sizeof(filename), "%s/%s", m_dev_path, path);

	FILE *pFile = fopen(filename, "w");

	if (pFile == nullptr) {
		DF_LOG_ERR("Unable to open file: %s", filename);
		return -1;
	}

	int result = fprintf(pFile, "%d", value);

	if (result < 0) {
		DF_LOG_ERR("Write error");
	}

	if (fclose(pFile) != 0) {
		DF_LOG_ERR("Unable to close file: %s", filename);
		return -1;
	}

	return 0;
}
