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
#include <list>
#include <vector>
#include <pthread.h>
#include <errno.h>
#include "DriverFramework.hpp"
#include "DevObj.hpp"
#include "DevMgr.hpp"
#include "SyncObj.hpp"

// Used for backtrace
#ifdef DF_ENABLE_BACKTRACE
#include <stdlib.h>
#include <execinfo.h>
#endif

#if defined(__APPLE__) && defined(__MACH__)
#include <mach/mach_time.h>
#define MAC_NANO (+1.0E-9)
#define MAC_GIGA (UINT64_C(1000000000))
#else
#include <time.h>
#endif

#define SHOW_STATS 0

namespace DriverFramework {
//-----------------------------------------------------------------------
// Types
//-----------------------------------------------------------------------
class WorkItem
{
public:
	WorkItem()
	{
		resetStats();
	}
	~WorkItem() {}

	void schedule();
	void updateStats(unsigned int cur_usec);
	void resetStats();
	void dumpStats();

	void set(WorkCallback callback, void *arg, uint32_t delay)
	{
		m_arg = arg;
		m_queue_time = 0;
		m_callback = callback;
		m_delay =delay;
		m_in_use = false;

		resetStats();
	}

	void *		m_arg;
	uint64_t	m_queue_time;
	WorkCallback	m_callback;
	uint32_t	m_delay;
	//WorkHandle 	m_handle;

	// statistics
	unsigned long 	m_last;
	unsigned long 	m_min;
	unsigned long 	m_max;
	unsigned long 	m_total;
	unsigned long 	m_count;

	bool		m_in_use = false;
};

class HRTWorkQueue : public DisableCopy
{
public:
	static HRTWorkQueue *instance(void);

	static int initialize(void);
	static void finalize(void);

	void scheduleWorkItem(WorkHandle &wh);
	void unscheduleWorkItem(WorkHandle &wh);

	void shutdown(void);
	void enableStats(bool enable);
	void clearAll();

	static void *process_trampoline(void *);

private:
	HRTWorkQueue(void) {}
	~HRTWorkQueue(void) {}

	void process(void);

	void hrtLock(void);
	void hrtUnlock(void);

	std::list<unsigned int>	m_work_list;

	bool m_enable_stats = false;
	bool m_exit_requested = false;

	static HRTWorkQueue *m_instance;
};

};

using namespace DriverFramework;

//-----------------------------------------------------------------------
// Static Variables
//-----------------------------------------------------------------------

static uint64_t g_timestart = 0;
static pthread_t g_tid;

static pthread_mutex_t g_framework_exit = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t g_hrt_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t g_reschedule_cond = PTHREAD_COND_INITIALIZER;
static pthread_cond_t g_framework_cond = PTHREAD_COND_INITIALIZER;

static std::vector<WorkItem> *g_work_items = nullptr;

static SyncObj *g_lock = nullptr;

//-----------------------------------------------------------------------
// Static Functions
//-----------------------------------------------------------------------

bool WorkMgr::isValid(const WorkHandle &h)
{
	return (g_work_items && (h.m_handle >=0) && ((unsigned int)h.m_handle < (*g_work_items).size()));
}

static uint64_t TSToABSTime(struct timespec *ts)
{
        uint64_t result;

        result = (uint64_t)(ts->tv_sec) * 1000000;
        result += ts->tv_nsec / 1000;

        return result;
}

#if defined(__APPLE__) && defined(__MACH__)
#include <time.h>
#include <sys/time.h>
#define CLOCK_REALTIME 0
static int clock_gettime(int clk_id, struct timespec* t)
{
	struct timeval now;
	int rv = gettimeofday(&now, NULL);

	if(rv) {
		return rv;
	}

	t->tv_sec = now.tv_sec;
	t->tv_nsec = now.tv_usec * 1000;

	return 0;
}
#endif

int DriverFramework::clockGetRealtime(struct timespec *ts)
{
	return clock_gettime(CLOCK_REALTIME, ts);
}

//-----------------------------------------------------------------------
// Global Functions
//-----------------------------------------------------------------------
uint64_t DriverFramework::offsetTime(void)
{
	struct timespec ts = {};

	(void)clockGetRealtime(&ts);

	if (!g_timestart) {
		g_timestart = TSToABSTime(&ts);
	}

	// Time is in microseconds
	return TSToABSTime(&ts) - g_timestart;
}

timespec DriverFramework::offsetTimeToAbsoluteTime(uint64_t offset_time)
{
	uint64_t abs_time = offset_time + g_timestart;
	struct timespec ts = {};
	ts.tv_sec = abs_time / 1000000;
	ts.tv_nsec = (abs_time % 1000000) * 1000;

	return ts;
}

timespec DriverFramework::absoluteTimeInFuture(uint64_t time_ms)
{
	struct timespec ts;

	clockGetRealtime(&ts);

	uint64_t nsecs = ts.tv_nsec + time_ms*1000000;
	uint64_t secs = (nsecs/1000000000);
	
	ts.tv_sec += secs;
	ts.tv_nsec = nsecs - secs*1000000000;

	return ts;
}

void DriverFramework::backtrace()
{
	void *buffer[10];
	char **callstack;
	int bt_size;
	int idx;

	bt_size = ::backtrace(buffer, 10);
	callstack = ::backtrace_symbols(buffer, bt_size);

	DF_LOG_DEBUG("Backtrace: %d", bt_size);

	for (idx = 0; idx < bt_size; idx++) {
		DF_LOG_DEBUG("%s", callstack[idx]);
	}

	free(callstack);
}

//-----------------------------------------------------------------------
// Class Methods
//-----------------------------------------------------------------------

/*************************************************************************
  Framework
*************************************************************************/
void Framework::shutdown()
{
	// Free the HRTWorkQueue resources
	HRTWorkQueue::finalize();

	// Free the WorkMgr resources
	WorkMgr::finalize();

	// Free the DevMgr resources
	DevMgr::finalize();

	// allow Framework to exit
	pthread_mutex_lock(&g_framework_exit);
	pthread_cond_signal(&g_framework_cond);
	pthread_mutex_unlock(&g_framework_exit);
	delete g_lock;
	g_lock = nullptr;
}

int Framework::initialize()
{
	int ret = HRTWorkQueue::initialize();
	if (ret < 0) {
		return ret-10;
	}
	ret = DevMgr::initialize();
	if (ret < 0) {
		return ret-20;
	}
	ret = WorkMgr::initialize();
	if (ret < 0) {
		return ret-30;
	}
	return 0;
}

void Framework::waitForShutdown()
{
	// Block until shutdown requested
	pthread_mutex_lock(&g_framework_exit);
	pthread_cond_wait(&g_framework_cond, &g_framework_exit);
	pthread_mutex_unlock(&g_framework_exit);
}

/*************************************************************************
  WorkItem
*************************************************************************/
void WorkItem::updateStats(unsigned int cur_usec)
{
	unsigned long delay = (m_last == ~0x0UL) ? (cur_usec - m_queue_time) : (cur_usec - m_last);

	if (delay < m_min) {
		m_min = delay;
	}
	if (delay > m_max) {
		m_max = delay;
	}

	m_total += delay;
	m_count += 1;
	m_last = cur_usec;

#if SHOW_STATS == 1
	if ((m_count % 100) == 99) {
		dumpStats();
	}
#endif
}

void WorkItem::resetStats() 
{
	m_last = ~(uint64_t)0;
	m_min = ~(uint64_t)0;
	m_max = 0;
	m_total = 0;
	m_count = 0;
}

void WorkItem::dumpStats() 
{
	DF_LOG_DEBUG("Stats for callback=%p: count=%lu, avg=%lu min=%lu max=%lu", 
		m_callback, m_count, m_total/m_count, m_min, m_max);
}

/*************************************************************************
  HRTWorkQueue
*************************************************************************/
HRTWorkQueue *HRTWorkQueue::m_instance = NULL;

void *HRTWorkQueue::process_trampoline(void *arg)
{
	if (m_instance) {
		m_instance->process();
	}
	return NULL;
}

HRTWorkQueue *HRTWorkQueue::instance(void)
{
	return m_instance;
}

static int setRealtimeSched(pthread_attr_t &attr)
{
	sched_param param;

	int ret = pthread_attr_init (&attr);
	ret = (!ret) ? ret : pthread_attr_getschedparam (&attr, &param);

	param.sched_priority = sched_get_priority_max(SCHED_FIFO);

	ret = (!ret) ? ret : pthread_attr_setschedparam (&attr, &param);
	ret = (!ret) ? ret : pthread_attr_setinheritsched (&attr, PTHREAD_EXPLICIT_SCHED);
	return ret;
}

int HRTWorkQueue::initialize(void)
{
	DF_LOG_DEBUG("HRTWorkQueue::initialize");
	m_instance = new HRTWorkQueue();

	if (m_instance == nullptr) {
		return 1;
	}

	// Create a lock for handling the work queue
	if (pthread_mutex_init(&g_hrt_lock, NULL) != 0) {
		return 2;
	}

	pthread_attr_t attr;
	if(setRealtimeSched(attr)) {
		return 3;
	}

	// Create high priority worker thread
	if (pthread_create(&g_tid, &attr, process_trampoline, NULL)) {
		return 4;
	}
	return 0;
}

void HRTWorkQueue::finalize(void)
{
	DF_LOG_DEBUG("HRTWorkQueue::finalize");

	// Stop the HRT queue thread
	HRTWorkQueue *wq = HRTWorkQueue::instance();
	if (wq) {
		wq->shutdown();

		// Wait for work queue thread to exit
		pthread_join(g_tid, NULL);

		wq->clearAll();
		pthread_mutex_destroy(&g_hrt_lock);

		delete wq;
		wq = nullptr;
	}
}

void HRTWorkQueue::scheduleWorkItem(WorkHandle &wh)
{
	DF_LOG_DEBUG("HRTWorkQueue::scheduleWorkItem (%p)", &wh);
	// Handle is known to be valid
	hrtLock();
	WorkItem &item = (*g_work_items)[wh.m_handle];
	item.m_queue_time = offsetTime();
	item.m_in_use = true;
	m_work_list.push_back(wh.m_handle);
	pthread_cond_signal(&g_reschedule_cond);
	hrtUnlock();
}

void HRTWorkQueue::unscheduleWorkItem(WorkHandle &wh)
{
	DF_LOG_DEBUG("HRTWorkQueue::unscheduleWorkItem (%p)", &wh);
	hrtLock();
	std::list<unsigned int>::iterator it = m_work_list.begin();
	while (it != m_work_list.end()) {

		// remove all unscheduled items
		if ((*g_work_items)[(*it)].m_in_use == false) {
			it = m_work_list.erase(it);
			break;
		}
		else {
			++it;
		}
	}
	hrtUnlock();
}

void HRTWorkQueue::clearAll()
{
	DF_LOG_DEBUG("HRTWorkQueue::clearAll");
	hrtLock();
	std::list<unsigned int>::iterator it = m_work_list.begin();
	while (it != m_work_list.end()) {
		(*g_work_items)[(*it)].m_in_use = false;
		it = m_work_list.erase(it);
	}
	hrtUnlock();
}

void HRTWorkQueue::shutdown(void)
{
	DF_LOG_DEBUG("HRTWorkQueue::shutdown");
	hrtLock();
	m_exit_requested = true;
	pthread_cond_signal(&g_reschedule_cond);
	hrtUnlock();
}

void HRTWorkQueue::process(void)
{
	DF_LOG_DEBUG("HRTWorkQueue::process");
	std::list<unsigned int>::iterator work_itr;
	uint64_t next;
	uint64_t elapsed;
	uint64_t remaining;
	timespec ts;
	uint64_t now;

	while(!m_exit_requested) {
		DF_LOG_DEBUG("HRTWorkQueue::process In while");
		hrtLock();

		// Wake up every 10 sec if nothing scheduled
		next = 10000000;
		work_itr = m_work_list.begin();

		now = offsetTime();
		while ((!m_exit_requested) && (work_itr != m_work_list.end())) {
			DF_LOG_DEBUG("HRTWorkQueue::process work exists");
			now = offsetTime();
			unsigned int index = *work_itr;
			if (index < (*g_work_items).size()) {
				WorkItem &item = (*g_work_items)[*work_itr];
				elapsed = now - item.m_queue_time;
				//DF_LOG_DEBUG("now = %lu elapsed = %lu delay = %lu\n", now, elapsed, item.m_delay);

				if (elapsed >= item.m_delay) {

					DF_LOG_DEBUG("HRTWorkQueue::process do work (%p)", item.m_callback);
					item.updateStats(now);

					// reschedule work
					item.m_queue_time = offsetTime();
					item.m_in_use = true;

					hrtUnlock();
					item.m_callback(item.m_arg);
					hrtLock();

					// Start again from the top to ge rescheduled work
					work_itr = m_work_list.begin();
				} else {
					remaining = item.m_delay - elapsed;
					if (remaining < next) {
						next = remaining;
					}

					// try the next in the list
					++work_itr;
				}
			}
		}

		// pthread_cond_timedwait uses absolute time
		ts = offsetTimeToAbsoluteTime(now+next);
		
		DF_LOG_DEBUG("HRTWorkQueue::process waiting for work (%" PRIu64 ")", next);
		// Wait until next expiry or until a new item is rescheduled
		pthread_cond_timedwait(&g_reschedule_cond, &g_hrt_lock, &ts);
		hrtUnlock();
	}
}

void HRTWorkQueue::hrtLock()
{
	pthread_mutex_lock(&g_hrt_lock);
}

void HRTWorkQueue::hrtUnlock()
{
	pthread_mutex_unlock(&g_hrt_lock);
}

/*************************************************************************
  WorkMgr
*************************************************************************/
int WorkMgr::initialize()
{
	DF_LOG_DEBUG("WorkMgr::initialize");
	if (g_lock) {
		// Already initialized
		return 0;
	}
	g_lock = new SyncObj();
	if (g_lock == nullptr) {
		return -1;
	}
	g_work_items = new std::vector<WorkItem>(10);
	if (g_work_items == nullptr) {
		delete g_lock;
		g_lock = nullptr;
		return -2;
	}
	return 0;
}

void WorkMgr::finalize()
{
	DF_LOG_DEBUG("WorkMgr::finalize");
	if (!g_lock) {
		return;
	}

	g_lock->lock();
	std::vector<WorkItem>::iterator it = g_work_items->begin();
	while(it != g_work_items->end())
	{
		//verify not in use
		if (it->m_in_use) {
			DF_LOG_ERR("ERROR: no work items should be in use");
		}
		++it;
	}
	
	g_work_items->clear();
	delete g_work_items;
	g_work_items = nullptr;
	g_lock->unlock();
	delete g_lock;
	g_lock = nullptr;
}

void WorkMgr::getWorkHandle(WorkCallback cb, void *arg, uint32_t delay, WorkHandle &wh)
{
	DF_LOG_DEBUG("WorkMgr::getWorkHandle");
	if (!g_lock || !g_work_items) {
		wh.m_errno = ESRCH;
		wh.m_handle = -1;
		return;
	}

	g_lock->lock();

	// unschedule work and erase the handle if handle exists
	if (isValid(wh) && (*g_work_items)[wh.m_handle].m_in_use) {
		HRTWorkQueue::instance()->unscheduleWorkItem(wh);
	}
	else if (!isValid(wh)) {

		// find an available WorkItem
		unsigned int i=0;
		for(; i<g_work_items->size(); ++i) {
			if (!(*g_work_items)[i].m_in_use) {
				wh.m_handle = i;
				break;
			}
		}

		if (!isValid(wh)) {
			DF_LOG_ERR("warning - no WorkItems available, adding a work item\n");
			WorkItem wi;
			i = (*g_work_items).size();
			(*g_work_items).resize(i+1, wi);
			wh.m_handle = (int)i;
		}
	}

	if (isValid(wh)) {
	
		// Re-use the WorkItem
		WorkItem &item = (*g_work_items)[wh.m_handle];

		item.set(cb, arg, delay);
	}

	g_lock->unlock();
	wh.m_errno = 0;
}

int WorkMgr::releaseWorkHandle(WorkHandle &wh)
{
	DF_LOG_DEBUG("WorkMgr::releaseWorkHandle");
	if ((g_lock == nullptr) || (g_work_items == nullptr)) {
		wh.m_errno = ESRCH;
		wh.m_handle = -1;
		return -1;
	}
	if (wh.m_handle == -1) {
		setError(wh, EBADF);
		return -2;
	}

	int ret = 0;
	g_lock->lock();
	if (isValid(wh)) {
		if ((*g_work_items)[wh.m_handle].m_in_use) {
			(*g_work_items)[wh.m_handle].m_in_use = false;
			HRTWorkQueue::instance()->unscheduleWorkItem(wh);
			wh.m_handle = -1;
		}
		wh.m_errno = 0;
	}
	else {
		setError(wh, EBADF);
		ret = -1;
	}
	
	g_lock->unlock();
	return ret;
}

int WorkMgr::schedule(WorkHandle &wh)
{
	DF_LOG_DEBUG("WorkMgr::schedule");
	if ((g_lock == nullptr) || (g_work_items == nullptr)) {
		wh.m_errno = ESRCH;
		return -1;
	}
	if (wh.m_handle == -1) {
		wh.m_errno = EBADF;
		return -2;
	}

	DF_LOG_DEBUG("WorkMgr::schedule - checks ok");
	int ret = 0;
	g_lock->lock();
	if (isValid(wh)) {
		if ((*g_work_items)[wh.m_handle].m_in_use) {
			DF_LOG_ERR("WorkMgr::schedule can't schedule a handle that's in use");
			wh.m_errno = EBUSY;
			ret = -3;
		}
		else {
			DF_LOG_DEBUG("WorkMgr::schedule - do scedule");
			HRTWorkQueue::instance()->scheduleWorkItem(wh);
		}
	}
	else {
		wh.m_errno = EBADF;
		wh.m_handle = -1;
		ret = -4;
	}
	g_lock->unlock();
	wh.m_errno = 0;
	return ret;
}
void WorkMgr::setError(WorkHandle &h, int error)
{
	h.m_errno = error;
}


WorkHandle::~WorkHandle()
{
	WorkMgr::releaseWorkHandle(*this);
}
