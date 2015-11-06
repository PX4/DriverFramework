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
#include <time.h>
#include "DisableCopy.hpp"

#pragma once

// Show backtrace on error
#define DF_ENABLE_BACKTRACE 1

//-----------------------------------------------------------------------
// Macros
//-----------------------------------------------------------------------

// Substitute logging implemntation here
#define DF_LOG_INFO(FMT, ...) printf(FMT "\n", ##__VA_ARGS__)
#define DF_LOG_ERR(FMT, ...)  printf(FMT "\n", ##__VA_ARGS__)

namespace DriverFramework {

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
uint64_t offsetTime(void);

// convert offset time to absolute time
struct timespec offsetTimeToAbsoluteTime(uint64_t offset_time);

// convert offset time to absolute time
struct timespec absoluteTimeInFuture(uint64_t time_ms);

/**
 * Get the absolute time off the system realtime clock
 *
 * @param timespec the realtime time
 *
 * @return 0 if successful, nonzero else
 */
int clockGetRealtime(struct timespec *ts);

#ifdef DF_ENABLE_BACKTRACE
// Used to show a backtrace while running
void backtrace();
#endif

class Framework
{
public:
	// Initialize the driver framework
	// This function must be called before any of the functions below
	static int initialize(void);

	// Terminate the driver framework
	static void shutdown(void);

	// Block until shutdown requested
	static void waitForShutdown();
};

// 
class WorkMgr
{
public:
	// Interface functions
	static void getWorkHandle(WorkCallback cb, void *arg, uint32_t delay, WorkHandle& handle);
	static int releaseWorkHandle(WorkHandle &handle);
	static int schedule(WorkHandle &handle);
	static void setError(WorkHandle &h, int error);

private:
	friend class Framework;

	static bool isValid(const WorkHandle &h);
	static int initialize(void);
	static void finalize(void);
};

};

