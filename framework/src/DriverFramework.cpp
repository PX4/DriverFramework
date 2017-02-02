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
#include <errno.h>
#include <time.h>
#include "DriverFramework.hpp"
#include "DevObj.hpp"
#include "DevMgr.hpp"
#include "SyncObj.hpp"
#include "WorkItems.hpp"

#ifdef __DF_LINUX
#include <sys/prctl.h>
#endif

// Used for backtrace
#ifdef DF_ENABLE_BACKTRACE
#include <stdlib.h>
#include <execinfo.h>
#endif

#define SHOW_STATS 0

namespace DriverFramework
{
//-----------------------------------------------------------------------
// Types
//-----------------------------------------------------------------------

class HRTWorkQueue : public DisableCopy
{
public:
	static HRTWorkQueue &instance();

	int initialize();
	void finalize();

	void scheduleWorkItem(WorkHandle &wh);

	void shutdown();
	void enableStats(bool enable);

	static void *process_trampoline(void *);

private:
	HRTWorkQueue() {}
	~HRTWorkQueue() {}

	void process();

	bool 	m_enable_stats = false;

	SyncObj m_reschedule;
};


};

using namespace DriverFramework;

//-----------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------
namespace DriverFramework
{
RunStatus *g_run_status = nullptr;
};


//-----------------------------------------------------------------------
// Static Variables
//-----------------------------------------------------------------------

static pthread_t g_tid;

// QuRT C++ compiler does not support static initialization of classes
static SyncObj *g_framework;

//-----------------------------------------------------------------------
// Static Functions
//-----------------------------------------------------------------------

static uint64_t TsToAbstime(struct timespec *ts)
{
	uint64_t result;

	result = (uint64_t)(ts->tv_sec) * 1000000;
	result += ts->tv_nsec / 1000;

	return result;
}

static uint64_t getStartTime()
{
	static SyncObj *lock = nullptr;
	static uint64_t starttime = 0;

	if (!lock) {
		lock = new SyncObj;
	}

	struct timespec ts = {};

	lock->lock();

	int ret = absoluteTime(ts);

	if (ret != 0) {
		printf("ERROR: absoluteTime returned (%d)", ret);
		lock->unlock();
		return 0;
	}

	if (!starttime) {
		starttime = TsToAbstime(&ts);
	}

	lock->unlock();

	return starttime;
}

//-----------------------------------------------------------------------
// Global Functions
//-----------------------------------------------------------------------

// NOTE: DO NOT USE DF_LOG_XXX here as it will cause a recursive loop
uint64_t DriverFramework::offsetTime()
{
	struct timespec ts = {};

	int ret = absoluteTime(ts);

	if (ret != 0) {
		printf("ERROR: absoluteTime returned (%d)", ret);
		return 0;
	}

	// Time is in microseconds
	uint64_t result = TsToAbstime(&ts) - getStartTime();

	return result;
}

timespec DriverFramework::offsetTimeToAbsoluteTime(uint64_t offset_time)
{
	struct timespec ts = {};
	uint64_t abs_time = offset_time + getStartTime();
	ts.tv_sec = abs_time / 1000000;
	ts.tv_nsec = (abs_time % 1000000) * 1000;

	return ts;
}

#if DF_ENABLE_BACKTRACE
void DriverFramework::backtrace()
{
	void *buffer[10];
	char **callstack;
	int bt_size;
	int idx;

	bt_size = ::backtrace(buffer, 10);
	callstack = ::backtrace_symbols(buffer, bt_size);

	DF_LOG_INFO("Backtrace: %d", bt_size);

	for (idx = 0; idx < bt_size; idx++) {
		DF_LOG_INFO("%s", callstack[idx]);
	}

	free(callstack);
}
#endif

//-----------------------------------------------------------------------
// Class Methods
//-----------------------------------------------------------------------

/*************************************************************************
  RunStatus
*************************************************************************/
bool RunStatus::check()
{
	m_lock.lock();
	bool ret = m_run;
	m_lock.unlock();
	return ret;
}

void RunStatus::terminate()
{
	m_lock.lock();
	m_run = false;
	m_lock.unlock();
}

/*************************************************************************
  Framework
*************************************************************************/
void Framework::shutdown()
{
	// Free the HRTWorkQueue resources
	HRTWorkQueue::instance().finalize();

	// Free the WorkMgr resources
	WorkMgr::finalize();

	// Free the DevMgr resources
	DevMgr::finalize();

	// allow Framework to exit
	g_framework->lock();
	g_framework->signal();
	g_framework->unlock();
}

int Framework::initialize()
{
	DF_LOG_DEBUG("Framework::initialize");

	g_framework = new SyncObj;

	if (!g_framework) {
		DF_LOG_ERR("ERROR: falled to allocate g_framework");
		return -1;
	}

	g_run_status = new RunStatus;

	if (!g_run_status) {
		DF_LOG_ERR("g_run_status allocation failed");
		return -2;
	}

	struct timespec ts = {};

	int ret = absoluteTime(ts);

	if (ret != 0) {
		DF_LOG_ERR("ERROR: absoluteTime returned (%d)", ret);
		return -4;
	}

	ret = HRTWorkQueue::instance().initialize();

	if (ret < 0) {
		return ret - 10;
	}

	DF_LOG_DEBUG("Calling DevMgr::initialize");
	ret = DevMgr::initialize();

	if (ret < 0) {
		return ret - 20;
	}

	DF_LOG_DEBUG("Calling WorkMgr::initialize");
	ret = WorkMgr::initialize();

	if (ret < 0) {
		return ret - 30;
	}

	return 0;
}

void Framework::waitForShutdown()
{
	// Block until shutdown requested
	g_framework->lock();
	g_framework->waitOnSignal(0);
	g_framework->unlock();

	delete g_framework;
	g_framework = nullptr;
}

/*************************************************************************
  HRTWorkQueue
*************************************************************************/

#ifndef __DF_QURT
// pthread_setschedparam does not seem to work on QURT.
static void show_sched_settings()
{
	int policy;
	struct sched_param param;

	int ret = pthread_getschedparam(pthread_self(), &policy, &param);

	if (ret != 0) {
		DF_LOG_ERR("pthread_getschedparam failed (%d)", ret);
	}

	DF_LOG_DEBUG("pthread info: policy=%s priority=%d",
		     (policy == SCHED_OTHER) ? "SCHED_OTHER" :
		     (policy == SCHED_FIFO)  ? "SCHED_FIFO" :
		     (policy == SCHED_RR)    ? "SCHED_RR" :
		     "UNKNOWN",
		     param.sched_priority);
}

static int setRealtimeSched()
{
	int policy = SCHED_FIFO;
	sched_param param;

	param.sched_priority = 10;

	int ret = pthread_setschedparam(pthread_self(), policy, &param);

	show_sched_settings();

	return ret;
}
#endif

void *HRTWorkQueue::process_trampoline(void *arg)
{
	DF_LOG_DEBUG("HRTWorkQueue::process_trampoline");

#ifndef __DF_QURT
	int ret = setRealtimeSched();

	if (ret != 0) {
		DF_LOG_ERR("WARNING: setRealtimeSched failed (not run as root?)");
	}

#endif

#ifdef __DF_LINUX
	prctl(PR_SET_NAME, "DFWorker"); //set the thread name
#endif

	DF_LOG_DEBUG("process_trampoline %d", ret);

	instance().process();

	return nullptr;
}

HRTWorkQueue &HRTWorkQueue::instance()
{
	static HRTWorkQueue *instance = nullptr;

	if (!instance) {
		instance = new HRTWorkQueue();
	}

	return *instance;
}

int HRTWorkQueue::initialize()
{
	DF_LOG_DEBUG("HRTWorkQueue::initialize");

	pthread_attr_t attr;
	int ret = pthread_attr_init(&attr);

	if (ret != 0) {
		DF_LOG_ERR("pthread_attr_init failed");
		return -1;
	}

#ifdef __DF_QURT
	// Try to set a stack size. This stack size is later used in _measure() calls
	// in the sensor drivers, at least on QURT.
	const size_t stacksize = 3072;

	if (pthread_attr_setstacksize(&attr, stacksize) != 0) {
		DF_LOG_ERR("failed to set stack size of %lu bytes", stacksize);
		return -2;
	}

#endif

	// Create high priority worker thread
	if (pthread_create(&g_tid, &attr, process_trampoline, nullptr)) {
		return -3;
	}

	DF_LOG_DEBUG("pthread_create success");

	return 0;
}

void HRTWorkQueue::finalize()
{
	DF_LOG_DEBUG("HRTWorkQueue::finalize");

	shutdown();

	// Wait for work queue thread to exit
	pthread_join(g_tid, nullptr);
}

void HRTWorkQueue::scheduleWorkItem(WorkHandle &wh)
{
	DF_LOG_DEBUG("HRTWorkQueue::scheduleWorkItem (%p)", &wh);

	// Handle is known to be valid
	int ret = WorkItems::schedule(wh.m_handle);

	DF_LOG_DEBUG("WorkItems::schedule %d", ret);

	if (ret == 0) {
		wh.m_errno = 0;
		m_reschedule.lock();
		m_reschedule.signal();
		m_reschedule.unlock();

	} else if (ret == EBADF) {
		wh.m_errno = EBADF;
		wh.m_handle = -1;

	} else {
		wh.m_errno = ret;
	}

}

void HRTWorkQueue::shutdown()
{
	DF_LOG_DEBUG("HRTWorkQueue::shutdown");

	if (g_run_status) {
		g_run_status->terminate();
	}

	m_reschedule.lock();
	m_reschedule.signal();
	m_reschedule.unlock();
}

void HRTWorkQueue::process()
{
	DF_LOG_DEBUG("HRTWorkQueue::process");
	uint64_t next;
	uint64_t now;

	while (g_run_status && g_run_status->check()) {
		now = offsetTime();

		// Wake up every 10 sec if nothing scheduled
		next = now + 10000000;

		WorkItems::processExpiredWorkItems(next);

		now = offsetTime();
		DF_LOG_DEBUG("now=%" PRIu64, now);
#ifdef __DF_QURT

		// to accomodate sleep inaccuracy in the platform
		if (next > now + 200) {
#else

		if (next > now) {
#endif
			uint64_t wait_time_usec = next - now;

			DF_LOG_DEBUG("HRTWorkQueue::process waiting for work (%" PRIi64 "usec)", wait_time_usec);
			// Wait until next expiry or until a new item is rescheduled
			m_reschedule.lock();
			m_reschedule.waitOnSignal(wait_time_usec);
			m_reschedule.unlock();
			DF_LOG_DEBUG("Done wait");
		}

		DF_LOG_DEBUG("not waiting for work (%" PRIi64 "usec)", wait_time_usec);
	}
};

// This has to be in ths file to access HRTWorkQueue::scheduleWorkItem
int WorkMgr::schedule(DriverFramework::WorkHandle &wh)
{
	DF_LOG_DEBUG("WorkMgr::schedule");
	HRTWorkQueue::instance().scheduleWorkItem(wh);
	return (wh.m_errno == 0) ? 0 : -1;
}

