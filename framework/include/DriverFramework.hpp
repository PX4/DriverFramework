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

#include "DFLog.hpp"

#include <stdint.h>
#include <time.h>
#include "HandleObj.hpp"

#ifndef __DF_NUTTX
#include "WorkMgr.hpp"
#endif

#ifdef __DF_BBBLUE
#include <board_config.h>
#endif

#ifdef __DF_LINUX
// Show backtrace on error
#define DF_ENABLE_BACKTRACE 1
#endif

namespace DriverFramework
{

/**
 * Get the absolute time off the system realtime clock
 *
 * @param ts the realtime time
 *
 * @return 0 if successful, nonzero else
 */
int absoluteTime(struct timespec &ts);

// convert offset time to absolute time
int absoluteTimeInFuture(uint64_t time_us, struct timespec &ts);

/**
 * Get the absolute time off the system monotonic clock
 *
 * @param ts the realtime time
 *
 * @return 0 if successful, nonzero else
 */
int clockGetMonotonic(struct timespec &ts);


#ifdef DF_ENABLE_BACKTRACE
// Used to show a backtrace while running
void backtrace();
#endif

class Framework
{
public:
	// Initialize the driver framework
	// This function must be called before any of the functions below
	static int initialize();

	// Terminate the driver framework
	static void shutdown();

	// Block until shutdown requested
	static void waitForShutdown();
};

}
