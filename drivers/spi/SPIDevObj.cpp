/****************************************************************************
 *
 *   Copyright (C) 2016 Julian Oes. All rights reserved.
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

#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include "OSConfig.h"
#include "SPIDevObj.hpp"
#include "DevIOCTL.h"
#ifdef __QURT
#include "dev_fs_lib_spi.h"
#endif

using namespace DriverFramework;

#define DIR_READ    0x80
#define DIR_WRITE   0x00

SPIDevObj::~SPIDevObj()
{
}

int SPIDevObj::start()
{
	m_fd = ::open(m_dev_path, 0);
	if (m_fd < 0) {
		DF_LOG_ERR("SPIDevObj start failed");
		return m_fd;
	}

	return 0;
}

int SPIDevObj::stop()
{
	if (m_fd >=0) {
		int ret = ::close(m_fd);
		if (ret < 0) {
			DF_LOG_ERR("Error: SPIDevObj::stop failed on ::close()");
		}
	}
}


int SPIDevObj::readReg(DevHandle &h, uint8_t address, uint8_t &val)
{
	SPIDevObj *obj = DevMgr::getDevObjByHandle<SPIDevObj>(h);
	if (obj) {
		return obj->_readReg(address, val);
	}
	return -1;
}

int SPIDevObj::_readReg(uint8_t address, uint8_t &val)
{
#ifdef __QURT
	/* Save the address of the register to read from in the write buffer for the combined write. */
	struct dspal_spi_ioctl_read_write ioctl_write_read;
	uint8_t write_buffer[2];
	uint8_t read_buffer[2];

	write_buffer[0] = address | DIR_READ;
	write_buffer[1] = 0;
	ioctl_write_read.write_buffer = write_buffer;
	ioctl_write_read.write_buffer_length = 2;
	ioctl_write_read.read_buffer = read_buffer;
	ioctl_write_read.read_buffer_length = 2;

	int result = ::ioctl(m_fd, SPI_IOCTL_RDWR, &ioctl_write_read);
	if (result < 0) {
		DF_LOG_ERR("error: SPI combined read write failed: %d", result);
		return -1;
	}

	val = read_buffer[1];
	return 0;
#else
	return -1;
#endif

}

int SPIDevObj::writeReg(DevHandle &h, uint8_t address, uint8_t val)
{
	SPIDevObj *obj = DevMgr::getDevObjByHandle<SPIDevObj>(h);
	if (obj) {
		return obj->_writeReg(address, val);
	}
	return -1;
}

int SPIDevObj::writeRegVerified(DevHandle &h, uint8_t address, uint8_t val)
{
	SPIDevObj *obj = DevMgr::getDevObjByHandle<SPIDevObj>(h);
	if (obj) {
		int result;
		uint8_t read_val = ~val;
		int retries = 5;
		while(retries) {
			result =  obj->_writeReg(address, val);
			if (result < 0) {
				--retries;
				continue;
			}
			result = obj->_readReg(address, read_val);
			if (result < 0 || read_val != val) {
                                --retries;
                                continue;
			}
		}
		if (val == read_val) {
			return 0;
		}
		else {
			DF_LOG_ERR("error: SPI write verify failed: %d", errno);
		}
	}
	return -1;
}


int SPIDevObj::_writeReg(uint8_t address, uint8_t val)
{
	uint8_t write_buffer[2];

	write_buffer[0] = address | DIR_WRITE;
	write_buffer[1] = val;

	/* Save the address of the register to read from in the write buffer for the combined write. */
	int bytes_written = ::write(m_fd, (char *) write_buffer, 2);
	if (bytes_written != 2) {
		DF_LOG_ERR("Error: SPI write failed. Reported %d bytes written (%d)", bytes_written, errno);
		return -1;
	}

	return 0;
}

int SPIDevObj::bulkRead(DevHandle &h, uint8_t address, uint8_t* out_buffer, int length)
{
#ifdef __QURT
	int result = 0;
	int transfer_bytes = 1 + length; // first byte is address
	struct dspal_spi_ioctl_read_write ioctl_write_read;
	uint8_t write_buffer[transfer_bytes];
	uint8_t read_buffer[transfer_bytes];

	write_buffer[0] = address | DIR_READ;

	ioctl_write_read.read_buffer = out_buffer;
	ioctl_write_read.read_buffer_length = transfer_bytes;
	ioctl_write_read.write_buffer = write_buffer;
	ioctl_write_read.write_buffer_length = transfer_bytes;
	result = h.ioctl(SPI_IOCTL_RDWR, (unsigned long)&ioctl_write_read);
	if (result != transfer_bytes)
	{
		DF_LOG_ERR("bulkRead error %d (%d)", result, h.getError());
		return result;
	}

	memcpy(out_buffer, &read_buffer[1], transfer_bytes-1);

	return 0;
#else
	return -1;
#endif
}

int SPIDevObj::_bulkRead(uint8_t address, uint8_t* out_buffer, int length)
{
#ifdef __QURT
	int result = 0;
	int transfer_bytes = 1 + length; // first byte is address

	struct dspal_spi_ioctl_read_write ioctl_write_read;
	uint8_t write_buffer[transfer_bytes];
	uint8_t read_buffer[transfer_bytes];

	::memset(write_buffer, 0, sizeof(write_buffer));
	write_buffer[0] = address | DIR_READ;

	ioctl_write_read.read_buffer = read_buffer;
	ioctl_write_read.read_buffer_length = transfer_bytes;
	ioctl_write_read.write_buffer = write_buffer;
	ioctl_write_read.write_buffer_length = transfer_bytes;
	result = ::ioctl(m_fd, SPI_IOCTL_RDWR, &ioctl_write_read);
	if (result != transfer_bytes)
	{
		DF_LOG_ERR("bulkRead error %d", result);
		return result;
	}

	memcpy(out_buffer, &read_buffer[1], transfer_bytes-1);

	return 0;
#else
	return -1;
#endif
}

int SPIDevObj::setLoopbackMode(DevHandle &h, bool enable)
{
#ifdef __QURT
	struct dspal_spi_ioctl_loopback loopback;

	loopback.state = enable ? SPI_LOOPBACK_STATE_ENABLED : SPI_LOOPBACK_STATE_DISABLED;
	return h.ioctl(SPI_IOCTL_LOOPBACK_TEST, (unsigned long)&loopback);
#else
	return -1;
#endif
}

int SPIDevObj::setBusFrequency(DevHandle &h, SPI_FREQUENCY freq_hz)
{
#ifdef __QURT
	struct dspal_spi_ioctl_set_bus_frequency bus_freq;
	bus_freq.bus_frequency_in_hz = freq_hz;
	return h.ioctl(SPI_IOCTL_SET_BUS_FREQUENCY_IN_HZ, (unsigned long)&bus_freq);
#else
	return -1;
#endif
}

int SPIDevObj::_setBusFrequency(SPI_FREQUENCY freq_hz)
{
#ifdef __QURT
	struct dspal_spi_ioctl_set_bus_frequency bus_freq;
	bus_freq.bus_frequency_in_hz = freq_hz;
	return ::ioctl(m_fd, SPI_IOCTL_SET_BUS_FREQUENCY_IN_HZ, &bus_freq);
#else
	return -1;
#endif
}
