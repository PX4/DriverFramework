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
#include "DriverFramework.hpp"
#ifdef __DF_QURT
#include "dspal_time.h"
#endif
#include <time.h>

#if defined(__PX4_POSIX) || defined(__PX4_QURT)
#include <px4_time.h>
#define df_clock_gettime px4_clock_gettime
#else
#define df_clock_gettime clock_gettime
#endif

// Only used for macOS unit test, not for SITL.
#if defined(__DF_APPLE_LEGACY) && !defined(__PX4_DARWIN)
#include <sys/time.h>
static int clock_gettime(int clk_id, struct timespec *t)
{
	struct timeval now;
	int rv = gettimeofday(&now, nullptr);

	if (rv) {
		return rv;
	}

	t->tv_sec = now.tv_sec;
	t->tv_nsec = now.tv_usec * 1000;

	return 0;
}
#endif

using namespace DriverFramework;

//-----------------------------------------------------------------------
// Global Functions
//-----------------------------------------------------------------------

int DriverFramework::absoluteTime(struct timespec &ts)
{
// On NuttX we use CLOCK_REALTIME anyway.
// On macOS we can only use CLOCK_REALTIME unless we're using the lockstep
// scheduler.
#if defined(__DF_NUTTX) || (defined(__DF_APPLE) && !defined(ENABLE_LOCKSTEP_SCHEDULER))

#ifndef CLOCK_REALTIME
#define CLOCK_REALTIME 0
#endif
	// CLOCK_MONOTONIC not available on NuttX or OSX
	return df_clock_gettime(CLOCK_REALTIME, &ts);
#else

#ifndef CLOCK_MONOTONIC
#define CLOCK_MONOTONIC 1
#endif
	return df_clock_gettime(CLOCK_MONOTONIC, &ts);
#endif
}

int DriverFramework::absoluteTimeInFuture(uint64_t time_us, struct timespec &ts)
{
	int ret = absoluteTime(ts);

	if (ret == 0) {
		uint64_t nsecs = ts.tv_nsec + time_us * 1000;
		uint64_t secs = (nsecs / 1000000000);

		ts.tv_sec += secs;
		ts.tv_nsec = nsecs - secs * 1000000000;
	}

	return ret;
}
