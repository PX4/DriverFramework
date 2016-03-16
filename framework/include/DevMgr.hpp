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
#include <stdio.h>
#include <stdint.h>
#include <sys/types.h>
#include <time.h>
#include "HandleObj.hpp"
#include "DFList.hpp"

#pragma once

#define NO_VERIFY 1 // Use fast method to get Driver Obj by Handle

namespace DriverFramework
{

// Forward class declarations
class DevMgr;
class DevObj;

class DevHandle : public HandleObj
{
public:
	DevHandle() {}

	virtual ~DevHandle();

	int ioctl(unsigned long cmd, unsigned long arg);
	ssize_t read(void *buf, size_t len);
	ssize_t write(const void *buf, size_t len);

private:
	friend DevMgr;
};

typedef DFPointerList UpdateList;



// DevMgr Is initalized by DriverFramework::initialize()
class DevMgr
{
public:

#ifndef __DF_NUTTX
	static int registerDriver(DevObj *obj);
	static void unregisterDriver(DevObj *obj);

	static DevObj *getDevObjByID(union DeviceId id);

	template <typename T>
	static T *getDevObjByHandle(DevHandle &handle)
	{
		if (!m_initialized || !handle.isValid()) {
			return nullptr;
		}

#ifdef NO_VERIFY
		return reinterpret_cast<T *>(handle.m_handle);
#else
		return dynamic_cast<T *>(_getDevObjByHandle(handle));
#endif
	}

	// Called by DevObj to notify threads waiting on an update
	static void updateNotify(DevObj &obj);

#endif

	static void getHandle(const char *dev_path, DevHandle &handle);
	static void releaseHandle(DevHandle &handle);

	// Similar to poll
	static int waitForUpdate(UpdateList &in_set, UpdateList &out_set, unsigned int timeout_ms);

	static void setDevHandleError(DevHandle &h, int error);

	static int getNextDeviceName(unsigned int &index, const char **instancename);
private:
	friend class Framework;

	DevMgr();
	~DevMgr();

	static int initialize(void);
	static void finalize(void);

#ifndef __DF_NUTTX
	static DevObj *_getDevObjByHandle(DevHandle &handle);

	static bool m_initialized;
#endif
};

};
