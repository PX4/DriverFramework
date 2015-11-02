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

#define DRIVER_MAX_INSTANCES 5

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
	DriverObj(const char *name, const char *dev_base_path, DeviceBusType bus_type, unsigned int sample_interval) :
		m_name(name),
		m_dev_base_path(dev_base_path),
		m_driver_instance(-1),
		m_refcount(0),
		m_sample_interval(sample_interval)
	{
		m_id.dev_id_s.bus = 0;
		m_id.dev_id_s.address = 0;
		m_id.dev_id_s.devtype = bus_type;
	}

	virtual int start(void)
	{
		if (m_driver_instance < 0) {
			int ret = DriverMgr::registerDriver(this);
			if (ret) {
				return ret;
			}
		}
		if (m_sample_interval && !m_work_handle) {
			m_work_handle = WorkMgr::create(measure, this, m_sample_interval);
			WorkMgr::schedule(m_work_handle);
		}
		return 0;
	}

	virtual int stop(void) {
		if (m_work_handle) {
			WorkMgr::destroy(m_work_handle);
			m_work_handle=0;
			DriverMgr::unregisterDriver(this);
		}
		return 0;
	}

	virtual ~DriverObj() 
	{
		if (isRegistered()) {
			DriverMgr::unregisterDriver(this);
		}
	}

	union DeviceId getId()
	{
		return m_id;
	}

	bool isRegistered()
	{
		return (m_driver_instance >= 0);
	}

	int getInstance();

	const std::string m_name;
	const std::string m_dev_base_path;
	std::string m_dev_instance_path;
	union DeviceId m_id;

	virtual void _measure() = 0;

	WorkHandle 	m_work_handle	= 0;

private:
	friend DriverMgr;

	static void measure(void *arg, const WorkHandle wh)
	{
		WorkMgr::schedule(wh);
		reinterpret_cast<DriverObj *>(arg)->_measure();
		
	}

	// Return -1 on failure, otherwise recount
	virtual int incrementRefcount()
	{
		if (isRegistered()) {
			int ret = start();
			if (ret < 0) {
				return -1;
			}
		}
		m_refcount++;
		return m_refcount;
	}

	// Return -1 on failure, otherwise recount
	virtual int decrementRefcount()
	{
		if (m_refcount) {
			m_refcount--;

			if (!m_refcount) {
				stop();
			}
		}
		else {
			return -1;
		}
		return m_refcount;
	}

	// Disallow copy
	DriverObj(const DriverObj&);

	int 		m_driver_instance;	// m_driver_instance = -1 when unregistered
	unsigned 	m_refcount;
	unsigned int 	m_sample_interval;
};

};
