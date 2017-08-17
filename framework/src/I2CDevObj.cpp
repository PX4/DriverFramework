/**********************************************************************
* Copyright (c) 2015 Mark Charlebois
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted (subject to the limitations in the
* disclaimer below) provided that the following conditions are met:
*
*  * Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
*  * Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the
*    distribution.
*
*  * Neither the name of Dronecode Project nor the names of its
*    contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
* NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
* GRANTED BY THIS LICENSE.  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
* HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
* WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
* MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
* BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
* WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
* OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
* IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*************************************************************************/
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include "OSConfig.h"
#include "I2CDevObj.hpp"
#include "DevIOCTL.h"
#ifdef __DF_QURT
#include "dev_fs_lib_i2c.h"
#elif defined(__DF_LINUX)
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#endif

using namespace DriverFramework;

int I2CDevObj::start()
{
	m_fd = ::open(m_dev_path, O_RDWR);

	if (m_fd < 0) {
		DF_LOG_ERR("Error: I2CDevObj::init failed on ::open() %s", m_dev_path);
		return m_fd;
	}

	return 0;
}

int I2CDevObj::stop()
{
	// close the device
	if (m_fd >= 0) {
		int ret = ::close(m_fd);
		m_fd = -1;

		if (ret < 0) {
			DF_LOG_ERR("Error: I2CDevObj::~I2CDevObj() failed on ::close()");
			return ret;
		}
	}

	return 0;
}

int I2CDevObj::readReg(DevHandle &h, uint8_t address, uint8_t *out_buffer, size_t length)
{
	I2CDevObj *obj = DevMgr::getDevObjByHandle<I2CDevObj>(h);

	if (obj) {
		return obj->_readReg(address, out_buffer, length);

	} else {
		return -1;
	}
}

int I2CDevObj::writeReg(DevHandle &h, uint8_t address, uint8_t *in_buffer, size_t length)
{
	I2CDevObj *obj = DevMgr::getDevObjByHandle<I2CDevObj>(h);

	if (obj) {
		return obj->_writeReg(address, in_buffer, length);

	} else {
		return -1;
	}
}

int I2CDevObj::_readReg(uint8_t address, uint8_t *out_buffer, size_t length)
{

	if (m_fd < 0) {
		DF_LOG_ERR("error: i2c bus is not yet opened");
		return -1;
	}

#ifdef __DF_QURT
	struct dspal_i2c_ioctl_combined_write_read ioctl_write_read;
	uint8_t write_buffer[1];

	/* Save the address of the register to read from in the write buffer for the combined write. */
	write_buffer[0] = address;
	ioctl_write_read.write_buf = write_buffer;
	ioctl_write_read.write_buf_len = 1;
	ioctl_write_read.read_buf = out_buffer;
	ioctl_write_read.read_buf_len = length;
	int bytes_read = ::ioctl(m_fd, I2C_IOCTL_RDWR, &ioctl_write_read);

	if (bytes_read != (ssize_t)length) {
		DF_LOG_ERR(
			"error: read register reports a read of %d bytes, but attempted to set %d bytes",
			bytes_read, length);
		return -1;
	}

	return 0;
#elif defined(__DF_LINUX)
	int result = _writeReg(address, nullptr, 0);

	if (result < 0) {
		return result;
	}

	result = _simple_read(out_buffer, length);

	if (result < 0) {
		return result;
	}

	return 0;
#else
	return -1;
#endif
}

int I2CDevObj::_readReg16(uint16_t address, uint16_t *out_buffer, size_t length)
{

	if (m_fd < 0) {
		DF_LOG_ERR("error: i2c bus is not yet opened");
		return -1;
	}

#ifdef __DF_QURT
	return -1;
#elif defined(__DF_LINUX)
	int result = _writeReg16(address, nullptr, 0);

	if (result < 0) {
		return result;
	}

	result = _simple_read((uint8_t *)out_buffer, length);

	if (result < 0) {
		return result;
	}

	return 0;
#else
	return -1;
#endif
}

// read from a register without ioctl
int I2CDevObj::_simple_read(uint8_t *out_buffer, size_t length)
{

	if (m_fd < 0) {
		DF_LOG_ERR("error: i2c bus is not yet opened");
		return -1;
	}

#if defined(__DF_QURT) || defined(__DF_LINUX)
	ssize_t bytes_read = 0;

	bytes_read = ::read(m_fd, out_buffer, length);

	if (bytes_read != (ssize_t)length) {
		DF_LOG_ERR("error: read register reports a read of %zd bytes, but attempted to set %zd bytes",
			   bytes_read, length);
		return -1;
	}

	return 0;
#else
	return -1;
#endif
}

int I2CDevObj::_writeReg(uint8_t address, uint8_t *in_buffer, size_t length)
{
#if defined(__DF_QURT) || defined(__DF_LINUX)
	unsigned retry_count = 0;

	if (m_fd < 0) {
		DF_LOG_ERR("error: i2c bus is not yet opened");
		return -1;
	}

	uint8_t write_buffer[length + 1];

	if (in_buffer) {
		memcpy(&write_buffer[1], in_buffer, length);
	}

	/* Save the address of the register to read from in the write buffer for the combined write. */
	write_buffer[0] = address;

	/*
	 * Verify that the length of the caller's buffer does not exceed the local stack
	 * buffer with one additional byte for the register ID.
	 */
	if (length + 1 > MAX_LEN_TRANSMIT_BUFFER_IN_BYTES) {
		DF_LOG_ERR("error: caller's buffer exceeds max len");
		return -1;
	}

	do {
		ssize_t bytes_written = ::write(m_fd, (char *) write_buffer, length + 1);

		if (bytes_written != (ssize_t)length + 1) {
			DF_LOG_ERR("Error: i2c write failed. Reported %zd bytes written",
				   bytes_written);

		} else {
			return 0;
		}

	} while (retry_count++ < _retries);

	return -1;
#else

	return -1;
#endif
}

int I2CDevObj::_writeReg16(uint16_t address, uint16_t *in_buffer, size_t length)
{
#if defined(__DF_QURT) || defined(__DF_LINUX)
	unsigned retry_count = 0;

	if (m_fd < 0) {
		DF_LOG_ERR("error: i2c bus is not yet opened");
		return -1;
	}

	uint8_t write_buffer[length + 2];

	if (in_buffer) {
		memcpy(&write_buffer[2], in_buffer, length);
	}

	/* Save the address of the register to read from in the write buffer for the combined write. */
	write_buffer[0] = (uint8_t)(address & 0xFF);
	write_buffer[1] = (uint8_t)(address >> 8);

	/*
	 * Verify that the length of the caller's buffer does not exceed the local stack
	 * buffer with one additional byte for the register ID.
	 */
	if (length + 2 > MAX_LEN_TRANSMIT_BUFFER_IN_BYTES) {
		DF_LOG_ERR("error: caller's buffer exceeds max len");
		return -1;
	}

	do {
		ssize_t bytes_written = ::write(m_fd, (char *) write_buffer, length + 2);

		if (bytes_written != (ssize_t)length + 2) {
			DF_LOG_ERR("Error: i2c write failed. Reported %zd bytes written",
				   bytes_written);

		} else {
			return 0;
		}

	} while (retry_count++ < _retries);

	return -1;
#else

	return -1;
#endif
}

int I2CDevObj::_setSlaveConfig(uint32_t slave_address, uint32_t bus_frequency_khz,
			       uint32_t transfer_timeout_usec)
{
#ifdef __DF_QURT
	struct dspal_i2c_ioctl_slave_config slave_config;
	memset(&slave_config, 0, sizeof(slave_config));
	slave_config.slave_address = slave_address;
	slave_config.bus_frequency_in_khz = bus_frequency_khz;
	slave_config.byte_transer_timeout_in_usecs = transfer_timeout_usec;
	return ::ioctl(m_fd, I2C_IOCTL_SLAVE, &slave_config);

#elif defined(__DF_LINUX)
	return ioctl(m_fd, I2C_SLAVE, slave_address);

#else
	return -1;
#endif
}
