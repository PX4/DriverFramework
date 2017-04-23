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
#pragma once

#include <stdint.h>
#include <time.h>
#ifdef __DF_QURT
#include <dspal_time.h>
#endif
#include "HandleObj.hpp"

namespace DriverFramework
{

// Types
class WorkHandle : public IntHandleObj
{
public:
	WorkHandle() {}

	virtual ~WorkHandle();
	friend WorkMgr;
};

typedef void (*WorkCallback)(void *arg);

// Get the offset time from startup
uint64_t offsetTime();

// convert offset time to absolute time
struct timespec offsetTimeToAbsoluteTime(uint64_t offset_time);

#ifdef DF_ENABLE_BACKTRACE
// Used to show a backtrace while running
void backtrace();
#endif

class Framework;

class WorkMgr
{
public:
	// Interface functions
	static void getWorkHandle(WorkCallback cb, void *arg, uint32_t delay_usec, WorkHandle &handle);
	static void releaseWorkHandle(WorkHandle &handle);
	static int schedule(WorkHandle &handle);
	static void setError(WorkHandle &h, int error);

private:
	friend class Framework;

	static bool isValidHandle(const WorkHandle &h);
	static int initialize();
	static void finalize();

	static bool m_initialized;
};

}

