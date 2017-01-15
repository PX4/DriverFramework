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
#include <pthread.h>
#include "DriverFramework.hpp"
#include "SyncObj.hpp"

#define DEBUG(FMT, ...)
//#define DEBUG(FMT, ...) printf(FMT, __VA_ARGS__)

using namespace DriverFramework;

SyncObj::SyncObj()
{
	// Cannot use recursive mutex for pthread_cond_timedwait in DSPAL
	initMutex(m_lock);

	pthread_condattr_t condattr;
	pthread_condattr_init(&condattr);

// CLOCK_MONOTONIC is not available on OSX and NuttX
// CLOCK_MONOTONIC is the default on QuRT so it need not be explicitly set
#if !defined(__DF_QURT) && !(defined(__APPLE__) && defined(__MACH__)) && !defined(__DF_NUTTX)

	// Configure the pthread_cond_timedwait to use the monotonic clock
	// because we don't want time skews to influence the scheduling.
	if (0 != pthread_condattr_setclock(&condattr, CLOCK_MONOTONIC)) {
		DF_LOG_ERR("ERROR: Failed to initialize m_new_data_cond attr");
	}

#endif

	pthread_cond_init(&m_new_data_cond, &condattr);
}

SyncObj::~SyncObj()
{
}

void SyncObj::lock()
{
	DEBUG("%p lock\n", this);
	pthread_mutex_lock(&m_lock);
}

void SyncObj::unlock()
{
	DEBUG("%p unlock\n", this);
	pthread_mutex_unlock(&m_lock);
}

// waitOnSignal must be called inside a lock()
// of this object
int SyncObj::waitOnSignal(unsigned long timeout_us)
{
	int ret;
	DEBUG("wait %p %lu us\n", &m_new_data_cond, timeout_us);

	if (timeout_us) {
		struct timespec ts;
		ret = absoluteTimeInFuture(timeout_us, ts);
		ret = ret ? ret : pthread_cond_timedwait(&m_new_data_cond, &m_lock, &ts);

	} else {
		ret = pthread_cond_wait(&m_new_data_cond, &m_lock);
	}

	return ret;
}

// signal must be called inside a lock()
// of this object
void SyncObj::signal()
{
	DEBUG("signal %p\n", &m_new_data_cond);
	pthread_cond_signal(&m_new_data_cond);
}

namespace DriverFramework
{

int initMutex(pthread_mutex_t &mutex)
{
	pthread_mutexattr_t attr;

	int rv = pthread_mutexattr_init(&attr);

	if (rv != 0) {
		DF_LOG_ERR("pthread_mutexattr_init failed");
		return -1;
	}

#ifndef __DF_NUTTX
	rv = pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_NORMAL);

	if (rv != 0) {
		DF_LOG_ERR("pthread_mutexattr_settype failed");
		return -1;
	}

#endif
	rv = pthread_mutex_init(&mutex, &attr);

	if (rv != 0) {
		DF_LOG_ERR("pthread_mutex_init failed");
		return -1;
	}

	return 0;
}
};
