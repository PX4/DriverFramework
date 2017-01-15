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
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include "DevObj.hpp"

#pragma once

namespace DriverFramework
{

class I2CDevHandle : public DevHandle
{
public:
	I2CDevHandle() :
		DevHandle()
	{}
	virtual ~I2CDevHandle();
};

class I2CDevObj : public DevObj
{
public:
	I2CDevObj(const char *name, const char *dev_path, const char *dev_class_path, unsigned int sample_interval_usec) :
		DevObj(name, dev_path, dev_class_path, DeviceBusType_I2C, sample_interval_usec),
		m_fd(-1),
		_retries(0)
	{
		m_id.dev_id_s.bus = DeviceBusType_I2C;
	}

	virtual ~I2CDevObj();

	virtual int start();
	virtual int stop();

	static int readReg(DevHandle &h, uint8_t address, uint8_t *out_buffer, int length);
	static int writeReg(DevHandle &h, uint8_t address, uint8_t *in_buffer, int length);

protected:
	int _readReg(uint8_t address, uint8_t *out_buffer, int length);
	int _writeReg(uint8_t address, uint8_t *out_buffer, int length);
	int _readReg16(uint16_t address, uint16_t *out_buffer, int length);
	int _writeReg16(uint16_t address, uint16_t *out_buffer, int length);

	// read from a register without ioctl
	int _simple_read(uint8_t *out_buffer, int length);

	int _setSlaveConfig(uint32_t slave_address, uint32_t bus_frequency_khz,
			    uint32_t transfer_timeout_usec);

	int m_fd;
	unsigned _retries;
};

};
