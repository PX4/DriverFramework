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
#include "DriverFramework.hpp"
#include "DFList.hpp"
#include "DevMgr.hpp"
#include "SyncObj.hpp"
#include "DisableCopy.hpp"
#include <stdint.h>

#pragma once

#if __DF_NUTTX
#error DevObj not supported in NuttX
#endif

#define DRIVER_MAX_INSTANCES 5

namespace DriverFramework
{

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

class DevObj : public DisableCopy
{
public:
	DevObj(const char *name, const char *dev_path, const char *dev_class_path, DeviceBusType bus_type,
	       unsigned int sample_interval_usec);

	virtual int init();

	virtual int start(void);

	virtual int stop(void);

	void setSampleInterval(unsigned int sample_interval_usecs);

	virtual ~DevObj();

	union DeviceId getId()
	{
		return m_id;
	}

	bool isRegistered()
	{
		return (m_driver_instance >= 0);
	}

	int getInstance()
	{
		return m_driver_instance;
	}

	virtual int devIOCTL(unsigned long request, unsigned long arg);

	virtual ssize_t devRead(void *buf, size_t count);

	virtual ssize_t devWrite(const void *buf, size_t count);

	void updateNotify();

	// _measure() is the periodic callback that is called every
	// m_sample_interval_usecs
	virtual void _measure() = 0; // periodic callback

	static uint16_t swap16(uint16_t val) { return (val >> 8) | (val << 8); }

	const char 	 	*m_name;
	const char 	 	*m_dev_path;
	char 	 		*m_dev_class_path;
	char 	 		*m_dev_instance_path;
	unsigned int 		m_sample_interval_usecs;
	union DeviceId		m_id;

	WorkHandle 		m_work_handle;
	bool 			m_pub_blocked;

private:
	int addHandle(DevHandle &h);
	int removeHandle(DevHandle &h);

	friend DevMgr;

	static void measure(void *arg);

	// Disallow copy
	DevObj(const DevObj &);

	int 			m_driver_instance;	// m_driver_instance = -1 when unregistered
	DFPointerList		m_handles;
	SyncObj			m_handle_lock;
	unsigned 		m_refcount;
};

};
