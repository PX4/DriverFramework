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

	void set(WorkCallback callback, void *arg, uint32_t delay_usec)
	{
		m_arg = arg;
		m_queue_time = 0;
		m_callback = callback;
		m_delay_usec = delay_usec;
		m_in_use = false;

		resetStats();
	}

	void 		*m_arg;
	uint64_t	m_queue_time;
	WorkCallback	m_callback;
	uint32_t	m_delay_usec;
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

	DFUIntList	m_work_list;

	bool m_enable_stats = false;
	bool m_exit_requested = false;

	static HRTWorkQueue *m_instance;
};

class WorkItems : public DFManagedList<WorkItem>
{
public:
	WorkItems() :
		DFManagedList()
	{}

	virtual ~WorkItems()
	{}

	bool getAt(unsigned int index, WorkItem **item)
	{
		Index idx = nullptr;
		idx = next(idx);

		for (unsigned int i = 0; i < index; ++i) {
			if (idx == nullptr) {
				return false;
			}

			idx = next(idx);
		}

		*item = get(idx);
		return true;
	}
};

};

using namespace DriverFramework;

//-----------------------------------------------------------------------
// Static Variables
//-----------------------------------------------------------------------

static uint64_t g_timestart = 0;
static pthread_t g_tid;

static pthread_mutex_t g_framework_exit;
static pthread_mutex_t g_hrt_lock;
static pthread_mutex_t g_timestart_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t g_reschedule_cond = PTHREAD_COND_INITIALIZER;
static pthread_cond_t g_framework_cond = PTHREAD_COND_INITIALIZER;

static WorkItems *g_work_items = nullptr;

static SyncObj *g_lock = nullptr;

//-----------------------------------------------------------------------
// Static Functions
//-----------------------------------------------------------------------

// TODO FIXME: this seems conflicting with WorkHandle::isValid()
bool WorkMgr::isValidHandle(const WorkHandle &h)
{
	return ((h.m_handle >= 0) && ((unsigned int)h.m_handle < g_work_items->size()));
}

static uint64_t TsToAbstime(struct timespec *ts)
{
	uint64_t result;

	result = (uint64_t)(ts->tv_sec) * 1000000;
	result += ts->tv_nsec / 1000;

	return result;
}

//-----------------------------------------------------------------------
// Global Functions
//-----------------------------------------------------------------------
uint64_t DriverFramework::offsetTime(void)
{
	struct timespec ts = {};

	(void)clockGetMonotonic(&ts);

	pthread_mutex_lock(&g_timestart_lock);

	if (!g_timestart) {
		g_timestart = TsToAbstime(&ts);
	}

	// Time is in microseconds
	uint64_t result = TsToAbstime(&ts) - g_timestart;
	pthread_mutex_unlock(&g_timestart_lock);

	return result;
}

timespec DriverFramework::offsetTimeToAbsoluteTime(uint64_t offset_time)
{
	pthread_mutex_lock(&g_timestart_lock);
	uint64_t abs_time = offset_time + g_timestart;
	pthread_mutex_unlock(&g_timestart_lock);
	struct timespec ts = {};
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

	DF_LOG_DEBUG("Backtrace: %d", bt_size);

	for (idx = 0; idx < bt_size; idx++) {
		DF_LOG_DEBUG("%s", callstack[idx]);
	}

	free(callstack);
}
#endif

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
}

int Framework::initialize()
{
	DF_LOG_DEBUG("Framework::initialize");

	int ret = HRTWorkQueue::initialize();

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
	pthread_mutex_lock(&g_framework_exit);
	pthread_cond_wait(&g_framework_cond, &g_framework_exit);
	pthread_mutex_unlock(&g_framework_exit);
}

/*************************************************************************
  WorkItem
*************************************************************************/
void WorkItem::updateStats(unsigned int cur_usec)
{
	unsigned long delay_usec = (m_last == ~0x0UL) ? (cur_usec - m_queue_time) : (cur_usec - m_last);

	if (delay_usec < m_min) {
		m_min = delay_usec;
	}

	if (delay_usec > m_max) {
		m_max = delay_usec;
	}

	m_total += delay_usec;
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
	m_last = ~(unsigned long)0;
	m_min = ~(unsigned long)0;
	m_max = 0;
	m_total = 0;
	m_count = 0;
}

void WorkItem::dumpStats()
{
	DF_LOG_DEBUG("Stats for callback=%p: count=%lu, avg=%lu min=%lu max=%lu",
		     m_callback, m_count, m_total / m_count, m_min, m_max);
}

/*************************************************************************
  HRTWorkQueue
*************************************************************************/
HRTWorkQueue *HRTWorkQueue::m_instance = NULL;

void *HRTWorkQueue::process_trampoline(void *arg)
{
	DF_LOG_DEBUG("HRTWorkQueue::process_trampoline");

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

	int ret = pthread_attr_init(&attr);
	ret = (!ret) ? ret : pthread_attr_getschedparam(&attr, &param);

#ifndef __QURT
	param.sched_priority = sched_get_priority_max(SCHED_FIFO);

	ret = (!ret) ? ret : pthread_attr_setschedparam(&attr, &param);
	ret = (!ret) ? ret : pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
#endif
	return ret;
}

int HRTWorkQueue::initialize(void)
{
	DF_LOG_DEBUG("HRTWorkQueue::initialize");
	m_instance = new HRTWorkQueue();
	DF_LOG_DEBUG("Created HRTWorkQueue");

	if (m_instance == nullptr) {
		return -1;
	}

	DF_LOG_DEBUG("m_instance = %p", m_instance);
	DF_LOG_DEBUG("Calling pthread_mutex_init");

	// Create a lock for handling the work queue
	// Cannot use recursive mutex for pthread_cond_timedwait in DSPAL
	if (initMutex(g_hrt_lock) < 0) {
		return -2;
	}

	if (initMutex(g_framework_exit) < 0) {
		return -3;
	}

	DF_LOG_DEBUG("pthread_mutex_init success");

#if !defined(__QURT) && !(defined(__APPLE__) && defined(__MACH__))
	// QURT supposedly always uses CLOCK_MONOTONIC.
	// Also CLOCK_MONOTONIC is not available on Mac.
	pthread_condattr_t condattr;
	pthread_condattr_init(&condattr);

	// Configure the pthread_cond_timedwait to use the monotonic clock
	// because we don't want time skews to influence the scheduling.
	if (0 != pthread_condattr_setclock(&condattr, CLOCK_MONOTONIC)) {
		return -4;
	}

	if (0 != pthread_cond_init(&g_reschedule_cond, &condattr)) {
		return -5;
	}

#endif

	pthread_attr_t attr;

	if (setRealtimeSched(attr)) {
		return -6;
	}

	DF_LOG_DEBUG("setRealtimeSched success");

#ifdef __QURT
	// Try to set a stack size. This stack size is later used in _measure() calls
	// in the sensor drivers, at least on QURT.
	const size_t stacksize = 2048;

	if (pthread_attr_setstacksize(&attr, stacksize) != 0) {
		DF_LOG_ERR("failed to set stack size of %lu bytes", stacksize);
		return -7;
	}

#endif

	// Create high priority worker thread
	if (pthread_create(&g_tid, &attr, process_trampoline, NULL)) {
		return -8;
	}

	DF_LOG_DEBUG("pthread_create success");
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
	WorkItem *item;
	g_work_items->getAt(wh.m_handle, &item);
	item->m_queue_time = offsetTime();
	item->m_in_use = true;
	m_work_list.pushBack(wh.m_handle);
	pthread_cond_signal(&g_reschedule_cond);
	hrtUnlock();
}

void HRTWorkQueue::unscheduleWorkItem(WorkHandle &wh)
{
	DF_LOG_DEBUG("HRTWorkQueue::unscheduleWorkItem (%p)", &wh);
	hrtLock();
	DFUIntList::Index idx = nullptr;
	idx = m_work_list.next(idx);

	while (idx != nullptr) {
		// If we find it in the list at the current idx, let's go ahead and delete it.
		unsigned index;

		if (m_work_list.get(idx, index)) {

			if (index == (unsigned)wh.m_handle) {
				// remove unscheduled item
				WorkItem *item;

				if (!g_work_items->getAt(index, &item)) {
					DF_LOG_ERR("HRTWorkQueue::unscheduleWorkItem - invalid index");

				} else {
					DF_LOG_DEBUG("HRTWorkQueue::unscheduleWorkItem - 2");
					item->m_in_use = false;
					idx = m_work_list.erase(idx);
					// We're only unscheduling one item, so we can bail out here.
					break;
				}
			}
		}

		idx = m_work_list.next(idx);
	}

	hrtUnlock();
}

void HRTWorkQueue::clearAll()
{
	DF_LOG_DEBUG("HRTWorkQueue::clearAll");
	hrtLock();
	DFUIntList::Index idx = nullptr;
	idx = m_work_list.next(idx);

	while (idx != nullptr) {
		unsigned int index;
		m_work_list.get(idx, index);
		WorkItem *item;
		g_work_items->getAt(index, &item);
		item->m_in_use = false;
		idx = m_work_list.erase(idx);
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
	uint64_t next;
	uint64_t elapsed;
	uint64_t remaining;
	timespec ts;
	uint64_t now;

	while (!m_exit_requested) {
		DF_LOG_DEBUG("HRTWorkQueue::process In while");
		hrtLock();

		// Wake up every 10 sec if nothing scheduled
		next = 10000000;

		DFUIntList::Index idx = nullptr;
		idx = m_work_list.next(idx);
		now = offsetTime();

		while ((!m_exit_requested) && (idx != nullptr)) {
			DF_LOG_DEBUG("HRTWorkQueue::process work exists");
			now = offsetTime();
			unsigned int index;
			m_work_list.get(idx, index);

			if (index < g_work_items->size()) {
				WorkItem *item;
				g_work_items->getAt(index, &item);
				elapsed = now - item->m_queue_time;
				//DF_LOG_DEBUG("now = %lu elapsed = %lu delay = %luusec\n", now, elapsed, item.m_delay_usec);

				if (elapsed >= item->m_delay_usec) {

					DF_LOG_DEBUG("HRTWorkQueue::process do work (%p) (%u)", item, item->m_delay_usec);
					item->updateStats(now);

					// reschedule work
					item->m_queue_time = offsetTime();
					item->m_in_use = true;

					void *tmpptr = item->m_arg;
					hrtUnlock();
					item->m_callback(tmpptr);
					hrtLock();

					// Start again from the top to get rescheduled work
					idx = nullptr;
					idx = m_work_list.next(idx);

				} else {
					remaining = item->m_delay_usec - elapsed;

					if (remaining < next) {
						next = remaining;
					}

					// try the next in the list
					idx = m_work_list.next(idx);
				}
			}
		}

		// pthread_cond_timedwait uses absolute time
		ts = offsetTimeToAbsoluteTime(now + next);

#ifdef __QURT
		uint64_t now_later = offsetTime();
		int64_t diff = (int64_t)(now + next) - (int64_t)now_later;

		// TODO FIXME: sometimes timeouts < 100 us seem to hang, therefore this workaround
		if (diff > 100) {
#endif
			DF_LOG_DEBUG("HRTWorkQueue::process waiting for work (%" PRIi64 ")", diff);
			// Wait until next expiry or until a new item is rescheduled
			pthread_cond_timedwait(&g_reschedule_cond, &g_hrt_lock, &ts);
#ifdef __QURT
		}

#endif
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

	g_work_items = new WorkItems;

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
	DFPointerList::Index idx = nullptr;
	idx = g_work_items->next(idx);

	while (idx != nullptr) {
		WorkItem *wi = reinterpret_cast<WorkItem *>(g_work_items->get(idx));

		//verify not in use
		if (wi->m_in_use) {
			DF_LOG_ERR("ERROR: no work items should be in use");
		}

		idx = g_work_items->next(idx);
	}

	g_work_items->clear();
	delete g_work_items;
	g_work_items = nullptr;
	g_lock->unlock();
	delete g_lock;
	g_lock = nullptr;
}

void WorkMgr::getWorkHandle(WorkCallback cb, void *arg, uint32_t delay_usec, WorkHandle &wh)
{
	DF_LOG_DEBUG("WorkMgr::getWorkHandle");

	if (!g_lock) {
		wh.m_errno = ESRCH;
		wh.m_handle = -1;
		return;
	}

	g_lock->lock();

	// unschedule work and erase the handle if handle exists
	if (isValidHandle(wh)) {
		WorkItem *item;

		if (g_work_items->getAt(wh.m_handle, &item)) {
			if (item->m_in_use) {
				DF_LOG_DEBUG("Unscheduled work (%p) (%u)", item, item->m_delay_usec);
				HRTWorkQueue::instance()->unscheduleWorkItem(wh);
			}

		} else {
			DF_LOG_ERR("Could not unschedule work, couldn't match handle");
		}

	} else {

		// find an available WorkItem
		unsigned int i = 0;
		DFPointerList::Index idx = nullptr;
		idx = g_work_items->next(idx);

		while (idx != nullptr) {
			WorkItem *wi = reinterpret_cast<WorkItem *>(g_work_items->get(idx));

			if (!wi->m_in_use) {
				wh.m_handle = i;
				break;
			}

			++i;
			idx = g_work_items->next(idx);
		}

		// If no free WorkItems, add one to the end
		if (!isValidHandle(wh)) {
			g_work_items->pushBack(new WorkItem());
			wh.m_handle = g_work_items->size() - 1;
		}
	}

	if (isValidHandle(wh)) {

		// Re-use the WorkItem
		WorkItem *item;
		g_work_items->getAt(wh.m_handle, &item);

		item->set(cb, arg, delay_usec);
	}

	g_lock->unlock();
	wh.m_errno = 0;
}

int WorkMgr::releaseWorkHandle(WorkHandle &wh)
{
	DF_LOG_DEBUG("WorkMgr::releaseWorkHandle");

	if (g_lock == nullptr) {
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

	if (isValidHandle(wh)) {
		WorkItem *item;
		g_work_items->getAt(wh.m_handle, &item);
		HRTWorkQueue::instance()->unscheduleWorkItem(wh);
		wh.m_handle = -1;
		wh.m_errno = 0;

	} else {
		setError(wh, EBADF);
		ret = -1;
	}

	g_lock->unlock();
	return ret;
}

int WorkMgr::schedule(WorkHandle &wh)
{
	DF_LOG_DEBUG("WorkMgr::schedule");

	if (g_lock == nullptr) {
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

	if (isValidHandle(wh)) {
		WorkItem *item;

		if (g_work_items->getAt(wh.m_handle, &item)) {
			if (item->m_in_use) {
				DF_LOG_ERR("WorkMgr::schedule can't schedule a handle that's in use");
				wh.m_errno = EBUSY;
				ret = -3;

			} else {
				DF_LOG_DEBUG("WorkMgr::schedule - do schedule");
				HRTWorkQueue::instance()->scheduleWorkItem(wh);
			}

		} else {
			DF_LOG_ERR("couldn't find handle to schedule");
			wh.m_errno = EBADF;
			ret = -4;
		}

	} else {
		wh.m_errno = EBADF;
		wh.m_handle = -1;
		ret = -5;
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
