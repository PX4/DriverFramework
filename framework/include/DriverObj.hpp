/**********************************************************************
* Copyright (c) 2012 Lorenz Meier
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
#include <string>
#include "DriverFramework.hpp"
#include "DriverMgr.hpp"

#pragma once

namespace DriverFramework {

// Re-use Device ID types from PX4
enum DeviceBusType {
	DeviceBusType_UNKNOWN = 0,
	DeviceBusType_I2C     = 1,
	DeviceBusType_SPI     = 2,
	DeviceBusType_UAVCAN  = 3,
	DeviceBusType_VIRT    = 4,
};

/*
  broken out device elements. The bitfields are used to keep
  the overall value small enough to fit in a float accurately,
  which makes it possible to transport over the MAVLink
  parameter protocol without loss of information.
 */
struct DeviceStructure {
	enum DeviceBusType bus_type : 3;
		uint8_t bus: 5;    // which instance of the bus type
		uint8_t address;   // address on the bus (eg. I2C address)
		uint8_t devtype;   // device class specific device type
};

union DeviceId {
	struct DeviceStructure dev_id_s;
	uint32_t dev_id;
};

class DriverObj
{
public:
	DriverObj(std::string name, DeviceBusType bus_type) : 
		m_name(name),
		m_registered(false),
		m_refcount(0)
	{
		m_id.dev_id_s.bus = 0;
		m_id.dev_id_s.address = 0;
		m_id.dev_id_s.devtype = bus_type;
	}

	virtual ~DriverObj() 
	{
		if (isRegistered()) {
			DriverMgr::unRegisterDriver(this);
		}
	}

	virtual int start() = 0;
	virtual int stop() = 0;

	const std::string &getName()
	{
		return m_name;
	}

	union DeviceId getId()
	{
		return m_id;
	}

	bool isRegistered()
	{
		return m_registered;
	}

private:
	friend DriverMgr;

	void incrementRefcount()
	{
		m_refcount++;
	}

	void decrementRefcount()
	{
		m_refcount++;
		if (m_refcount) {
			m_refcount--;
		}
	}

	// Disallow copy
	DriverObj(const DriverObj&);

	std::string m_name;
	union DeviceId m_id;

	bool m_registered;
	unsigned m_refcount;
};

};
