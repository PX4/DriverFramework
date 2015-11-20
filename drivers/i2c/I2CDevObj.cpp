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
#include "dev_fs_lib_i2c.h"
#include "I2CDevObj.hpp"
#include "DevIOCTL.h"

using namespace DriverFramework;

int I2CDevObj::start()
{
	m_fd = ::open(m_dev_path, O_RDWR);
	if (m_fd >=0) {
		DevObj::start();
	}
	return (m_fd < 0) ? m_fd : 0;
}

int I2CDevObj::stop()
{
	int ret;
	if (m_fd >=0) {
		ret = ::close(m_fd);
		if (ret < 0) {
			DF_LOG_ERR("Error: I2CDevObj::stop failed on ::close()");
		}
	}
	ret = DevObj::stop();
	return ret;
}

int I2CDevObj::readReg(DevHandle &h, uint8_t address, uint8_t *out_buffer, int length)
{
	I2CDevObj *obj = DevMgr::getDevObjByHandle<I2CDevObj>(h);
	if (obj) {
		return obj->_readReg(address, out_buffer, length);
	}
	else {
		return -1;
	}
}

int I2CDevObj::writeReg(DevHandle &h, uint8_t address, uint8_t *in_buffer, int length)
{
	I2CDevObj *obj = DevMgr::getDevObjByHandle<I2CDevObj>(h);
	if (obj) {
		return obj->_writeReg(address, in_buffer, length);
	}
	else {
		return -1;
	}
}

int I2CDevObj::_readReg(uint8_t address, uint8_t *out_buffer, int length)
{
	struct dspal_i2c_ioctl_combined_write_read ioctl_write_read;
	uint8_t write_buffer[1];

	if (m_fd == 0) {
		DF_LOG_ERR("error: i2c bus is not yet opened");
		return -1;
	}

	/* Save the address of the register to read from in the write buffer for the combined write. */
	write_buffer[0] = address;
	ioctl_write_read.write_buf = write_buffer;
	ioctl_write_read.write_buf_len = 1;
	ioctl_write_read.read_buf = out_buffer;
	ioctl_write_read.read_buf_len = length;
	int bytes_written = ::ioctl(m_fd, I2C_IOCTL_RDWR, &ioctl_write_read);
	if (bytes_written != length) {
		DF_LOG_ERR(
			"error: read register reports a read of %d bytes, but attempted to set %d bytes",
			bytes_written, length);
		return -1;
	}

	return 0;
}

int I2CDevObj::_writeReg(uint8_t address, uint8_t *in_buffer, int length)
{
	uint8_t write_buffer[MAX_LEN_TRANSMIT_BUFFER_IN_BYTES];

	/*
	 * Verify that the length of the caller's buffer does not exceed the local stack
	 * buffer with one additional byte for the register ID.
	 */
	if (length + 1 > MAX_LEN_TRANSMIT_BUFFER_IN_BYTES) {
		DF_LOG_ERR("error: caller's buffer exceeds size of local buffer");
		return -1;
	}
	if (m_fd == 0) {
		DF_LOG_ERR("error: i2c bus is not yet opened");
		return -1;
	}

	/* Save the address of the register to read from in the write buffer for the combined write. */
	write_buffer[0] = address;
	memcpy(&write_buffer[1], in_buffer, length);
	int bytes_written = ::write(m_fd, (char *) write_buffer, length + 1);
	if (bytes_written != length + 1) {
		DF_LOG_ERR("Error: i2c write failed. Reported %d bytes written",
				bytes_written);
		return -1;
	}

	return 0;
}

