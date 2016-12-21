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

#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include "DevObj.hpp"

#pragma once

namespace DriverFramework
{

class SPIDevHandle : public DevHandle
{
public:
	SPIDevHandle() :
		DevHandle()
	{}
	virtual ~SPIDevHandle();
};

class SPIDevObj : public DevObj
{
public:
	enum SPI_FREQUENCY {
		SPI_FREQUENCY_320KHZ = 320000UL,
		SPI_FREQUENCY_1MHZ = 1000000UL,
		SPI_FREQUENCY_5MHZ = 5000000UL,
		SPI_FREQUENCY_10MHZ = 10000000UL,
		SPI_FREQUENCY_15MHZ = 15000000UL,
		SPI_FREQUENCY_20MHZ = 20000000UL,
	};

	SPIDevObj(const char *name, const char *dev_path, const char *dev_class_path, unsigned int sample_interval_usec) :
		DevObj(name, dev_path, dev_class_path, DeviceBusType_SPI, sample_interval_usec)
	{
		m_id.dev_id_s.bus = DeviceBusType_SPI;
	}

	virtual ~SPIDevObj();

	virtual int start();
	virtual int stop();

	static int readReg(DevHandle &h, uint8_t address, uint8_t &val);
	static int writeReg(DevHandle &h, uint8_t address, uint8_t val);
	static int writeRegVerified(DevHandle &h, uint8_t address, uint8_t val);
	static int bulkRead(DevHandle &h, uint8_t address, uint8_t *out_buffer, int length);
	static int setLoopbackMode(DevHandle &h, bool enable);
	static int setBusFrequency(DevHandle &h, SPI_FREQUENCY freq_hz);

protected:
	int _readReg(uint8_t address, uint8_t &val);
	int _writeReg(uint8_t address, uint8_t *in_buffer, uint16_t length);
	int _writeReg(uint8_t address, uint8_t val);
	int _modifyReg(uint8_t address, uint8_t clearbits, uint8_t setbits);

	int _bulkRead(uint8_t address, uint8_t *out_buffer, int length);
	int _setBusFrequency(SPI_FREQUENCY freq_hz);

	int m_fd = 0;
};

};
