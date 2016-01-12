/**********************************************************************
* Copyright (c) 2015 Julian Oes
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
* 3. Neither the name PX4 nor the names of its contributors may be
*    used to endorse or promote products derived from this software
*    without specific prior written permission.
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
#include "dev_fs_lib_spi.h"
#include "SPIDevObj.hpp"
#include "DevIOCTL.h"

using namespace DriverFramework;

int SPIDevObj::start()
{
    DF_LOG_INFO("Opening: %s", m_dev_path);
	m_fd = ::open(m_dev_path, 0);

	if (m_fd >=0) {
		DevObj::start();
	} else {
        DF_LOG_ERR("Error: SPIDevObj::start failed on ::start()");
    }
    DF_LOG_INFO("SPI open done");
	return (m_fd < 0) ? m_fd : 0;
}

int SPIDevObj::stop()
{
	int ret;
	if (m_fd >=0) {
		ret = ::close(m_fd);
		if (ret < 0) {
			DF_LOG_ERR("Error: SPIDevObj::stop failed on ::close()");
		}
	}
	ret = DevObj::stop();
	return ret;
}

int SPIDevObj::readReg(DevHandle &h, uint8_t address, uint8_t *out_buffer, int length)
{
	SPIDevObj *obj = DevMgr::getDevObjByHandle<SPIDevObj>(h);
	if (obj) {
		return obj->_readReg(address, out_buffer, length);
	}
	else {
		return -1;
	}
}

int SPIDevObj::writeReg(DevHandle &h, uint8_t address, uint8_t *in_buffer, int length)
{
	SPIDevObj *obj = DevMgr::getDevObjByHandle<SPIDevObj>(h);
	if (obj) {
		return obj->_writeReg(address, in_buffer, length);
	}
	else {
		return -1;
	}
}

int SPIDevObj::_readReg(uint8_t address, uint8_t *out_buffer, int length)
{

	if (m_fd == 0) {
		DF_LOG_ERR("error: SPI bus is not yet opened");
		return -1;
	}


	/* Save the address of the register to read from in the write buffer for the combined write. */
	struct dspal_spi_ioctl_read_write ioctl_write_read;
	ioctl_write_read.write_buffer = reinterpret_cast<void(*)>(address);
	ioctl_write_read.write_buffer_length = sizeof(address);
	ioctl_write_read.read_buffer = out_buffer;
	ioctl_write_read.read_buffer_length = length;

    int result = ::ioctl(m_fd, SPI_IOCTL_RDWR, &ioctl_write_read);
    // TODO: is < 0 an error or != 0?
    if (result < 0) {
		DF_LOG_ERR(
			"error: SPI write failed: %d", result);
		return -1;
    }

	return 0;
}

int SPIDevObj::_writeReg(uint8_t address, uint8_t *in_buffer, int length)
{
    uint8_t write_data_buffer[DSPAL_SPI_TRANSMIT_BUFFER_LENGTH];

	/*
	 * Verify that the length of the caller's buffer does not exceed the local stack
	 * buffer with one additional byte for the register ID.
	 */
	if (length + 1 > DSPAL_SPI_TRANSMIT_BUFFER_LENGTH) {
		DF_LOG_ERR("error: caller's buffer exceeds size of local buffer");
		return -1;
	}
	if (m_fd == 0) {
		DF_LOG_ERR("error: SPI bus is not yet opened");
		return -1;
	}

	/* Save the address of the register to read from in the write buffer for the combined write. */
	write_data_buffer[0] = address;
	memcpy(&write_data_buffer[1], in_buffer, length);
	int bytes_written = ::write(m_fd, (char *) write_data_buffer, length + 1);
	if (bytes_written != length/* + 1*/) {
		DF_LOG_ERR("Error: SPI write failed. Reported %d bytes written",
				bytes_written);
		return -1;
	}

	return 0;
}

