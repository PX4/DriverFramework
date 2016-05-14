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

class HRTWorkQueue : public DisableCopy
{
public:
	static HRTWorkQueue &instance(void);

	static int initialize(void);
	static void finalize(void);

	void scheduleWorkItem(WorkHandle &wh);

	void shutdown(void);
	void enableStats(bool enable);

	static void *process_trampoline(void *);

private:
	HRTWorkQueue(void) {}
	~HRTWorkQueue(void) {}

	void process(void);

	bool m_enable_stats = false;
};

class WorkItems
{
public:
	static bool isValidIndex(unsigned int index);

	static WorkItems &instance()
	{
		static WorkItems instance;
		return instance;
	}

	static int  getIndex(WorkCallback cb, void *arg, uint32_t delay_usec, unsigned int &index);
	static void processExpiredWorkItems(uint64_t &next);
	static int  schedule(unsigned int index);
	static void unschedule(unsigned int index);
	static void finalize();

private:
	WorkItems() {}

	virtual ~WorkItems()
	{}

	void removeItem(WorkHandle &wh);
	void addItem(WorkHandle &wh);

	// These version do not call m_lock.lock()
	int  _schedule(unsigned int index);
	void _unschedule(unsigned int index);
	void _finalize();
	void _processExpiredWorkItems(uint64_t &next);
	int  _getIndex(WorkCallback cb, void *arg, uint32_t delay_usec, unsigned int &index);
	bool _isValidIndex(unsigned int index);

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

	// The list of WorkItem only grows via push_back so the order is preserved and
	// a index from 0-n can be used to retrieve an entry via getAt to iterate through
	// the list to that point.
	// This is somewhat inefficuent but in general the depth of the list is expected
	// to be small
	bool getAt(unsigned int index, WorkItem **item)
	{
		DFManagedList<WorkItem>::Index idx = nullptr;
		idx = m_work_items.next(idx);

		for (unsigned int i = 0; i < index; ++i) {

			// if the index is invalid m_work_items.next() will return nullptr
			if (idx == nullptr) {
				return false;
			}

			idx = m_work_items.next(idx);
		}

		*item = m_work_items.get(idx);
		return true;
	}


	DFUIntList		m_work_list; 	// List of active work items
	DFManagedList<WorkItem> m_work_items;	// List of all created work items
	SyncObj			m_lock;
};

};

using namespace DriverFramework;

//-----------------------------------------------------------------------
// Static Variables
//-----------------------------------------------------------------------

static pthread_t g_tid;

static SyncObj 	g_framework;
static SyncObj 	g_reschedule;

static SyncObj 	g_lock_df;
static bool 	g_exit_requested = false;

bool WorkMgr::m_initialized = false;

//-----------------------------------------------------------------------
// Static Functions
//-----------------------------------------------------------------------

// TODO FIXME: this seems conflicting with WorkHandle::isValid()
bool WorkMgr::isValidHandle(const WorkHandle &h)
{
	return ((h.m_handle >= 0) && WorkItems::isValidIndex((unsigned int)h.m_handle));
}

bool WorkItems::isValidIndex(unsigned int index)
{
	WorkItems &inst = instance();

	inst.m_lock.lock();
	bool ret = inst._isValidIndex(index);
	inst.m_lock.unlock();
	return ret;
}

bool WorkItems::_isValidIndex(unsigned int index)
{
	return (index < m_work_items.size());
}

static uint64_t TsToAbstime(struct timespec *ts)
{
	uint64_t result;

	result = (uint64_t)(ts->tv_sec) * 1000000;
	result += ts->tv_nsec / 1000;

	return result;
}

static uint64_t getStartTime()
{
	static SyncObj lock;
	static uint64_t starttime = 0;

	struct timespec ts = {};

	lock.lock();
	int ret = absoluteTime(ts);
	if (ret != 0) {
		printf("ERROR: absoluteTime returned (%d)", ret);
		return 0;
	}

	if (!starttime) {
		starttime = TsToAbstime(&ts);
	}
	lock.unlock();

	return starttime;
}

//-----------------------------------------------------------------------
// Global Functions
//-----------------------------------------------------------------------

// NOTE: DO NOT USE DF_LOG_XXX here as it will cause a recursive loop
uint64_t DriverFramework::offsetTime(void)
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
	g_framework.lock();
	g_framework.signal();
	g_framework.unlock();
}

int Framework::initialize()
{
	DF_LOG_DEBUG("Framework::initialize");

	struct timespec ts = {};

	int ret = absoluteTime(ts);
	if (ret != 0) {
		DF_LOG_ERR("ERROR: absoluteTime returned (%d)", ret);
		return -1;
	}

	ret = HRTWorkQueue::initialize();

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
	g_framework.lock();
	g_framework.waitOnSignal(0);
	g_framework.unlock();
}

/*************************************************************************
  WorkItem
*************************************************************************/
void WorkItems::WorkItem::updateStats(unsigned int cur_usec)
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

void WorkItems::WorkItem::resetStats()
{
	m_last = ~(unsigned long)0;
	m_min = ~(unsigned long)0;
	m_max = 0;
	m_total = 0;
	m_count = 0;
}

void WorkItems::WorkItem::dumpStats()
{
	DF_LOG_DEBUG("Stats for callback=%p: count=%lu, avg=%lu min=%lu max=%lu",
		     m_callback, m_count, m_total / m_count, m_min, m_max);
}

/*************************************************************************
  HRTWorkQueue
*************************************************************************/
#include <unistd.h>
#include <sys/syscall.h>   /* For SYS_xxx definitions */
#include <sys/time.h>
#include <sys/resource.h>

static void show_sched_settings()
{
	int policy;
	struct sched_param param;

	int ret = pthread_getschedparam(pthread_self(), &policy, &param);
	if (ret != 0) {
		DF_LOG_ERR("pthread_getschedparam failed (%d)", ret);
	}

	DF_LOG_INFO("pthread info: policy=%s priority=%d",
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

void *HRTWorkQueue::process_trampoline(void *arg)
{
	DF_LOG_DEBUG("HRTWorkQueue::process_trampoline");

	int ret = setRealtimeSched();
	if (ret != 0) {
		DF_LOG_ERR("WARNING: setRealtimeSched failed (not run as root?)");
	}

	DF_LOG_INFO("process_trampoline %d", ret);

	instance().process();

	return NULL;
}

HRTWorkQueue &HRTWorkQueue::instance(void)
{
	static HRTWorkQueue instance;
	return instance;
}

int HRTWorkQueue::initialize(void)
{
	DF_LOG_DEBUG("HRTWorkQueue::initialize");
	DF_LOG_DEBUG("Calling pthread_mutex_init");

	pthread_attr_t attr;
	int ret = pthread_attr_init(&attr);

	if (ret != 0) {
		DF_LOG_ERR("pthread_attr_init failed");
		return -6;
	}

#ifdef __QURT
	// Try to set a stack size. This stack size is later used in _measure() calls
	// in the sensor drivers, at least on QURT.
	const size_t stacksize = 3072;

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

	instance().shutdown();

	// Wait for work queue thread to exit
	pthread_join(g_tid, NULL);
}

void HRTWorkQueue::scheduleWorkItem(WorkHandle &wh)
{
	DF_LOG_DEBUG("HRTWorkQueue::scheduleWorkItem (%p)", &wh);

	g_reschedule.lock();

	// Handle is known to be valid
	int ret = WorkItems::schedule(wh.m_handle);

	if (ret == 0) {
		wh.m_errno = 0;
		g_reschedule.signal();
	} else if (ret == EBADF) {
		wh.m_errno = EBADF;
		wh.m_handle = -1;
	} else {
		wh.m_errno = ret;
	}

	g_reschedule.unlock();
}

void WorkItems::finalize()
{
	WorkItems &inst = instance();

	inst.m_lock.lock();
	inst._finalize();
	inst.m_lock.unlock();
}

void WorkItems::_finalize()
{
	m_work_list.clear();
	m_work_items.clear();
}

void WorkItems::unschedule(unsigned int index)
{
	WorkItems &inst = instance();

	inst.m_lock.lock();
	inst._unschedule(index);
	inst.m_lock.unlock();
}

void WorkItems::_unschedule(unsigned int index)
{
	DFUIntList::Index idx = nullptr;
	idx = m_work_list.next(idx);

	while (idx != nullptr) {
		// If we find it in the list at the current idx, let's go ahead and delete it.
		unsigned cur_index;

		if (m_work_list.get(idx, cur_index)) {

			if (cur_index == index) {
				// remove unscheduled item
				WorkItem *item = nullptr;

				if (!getAt(index, &item)) {
					DF_LOG_ERR("HRTWorkQueue::unscheduleWorkItem - invalid index");

				} else {
					item->m_in_use = false;
					idx = m_work_list.erase(idx);
					// We're only unscheduling one item, so we can bail out here.
					break;
				}
			}
		}

		idx = m_work_list.next(idx);
	}
}

void HRTWorkQueue::shutdown(void)
{
	DF_LOG_DEBUG("HRTWorkQueue::shutdown");
	g_reschedule.lock();
	g_exit_requested = true;
	g_reschedule.waitOnSignal(0);
	g_reschedule.unlock();
}

void HRTWorkQueue::process(void)
{
	DF_LOG_DEBUG("HRTWorkQueue::process");
	uint64_t next;
	timespec ts;
	uint64_t now;

	while (!g_exit_requested) {
		now = offsetTime();

		// Wake up every 10 sec if nothing scheduled
		next = now + 10000000;

		WorkItems::processExpiredWorkItems(next);

		uint64_t TUNING_ADJUSTMENT = 150;

		now = offsetTime();
		next -= TUNING_ADJUSTMENT;
		if (next > now) {
			// pthread_cond_timedwait uses absolute time
			ts = offsetTimeToAbsoluteTime(next-TUNING_ADJUSTMENT);

			DF_LOG_DEBUG("HRTWorkQueue::process waiting for work (%" PRIi64 "usec)", next - offsetTime());
			// Wait until next expiry or until a new item is rescheduled
			g_reschedule.lock();
			g_reschedule.waitOnSignal(next - now);
			g_reschedule.unlock();
		}
	}
};

void WorkItems::processExpiredWorkItems(uint64_t &next)
{
	instance()._processExpiredWorkItems(next);
}

void WorkItems::_processExpiredWorkItems(uint64_t &next)
{
	DF_LOG_DEBUG("WorkItems::processExpiredWorkItems");
	uint64_t elapsed;
	uint64_t now;

	m_lock.lock();

	DFUIntList::Index idx = nullptr;
	idx = m_work_list.next(idx);

	while ((!g_exit_requested) && (idx != nullptr)) {
		DF_LOG_DEBUG("HRTWorkQueue::process work exists");
		unsigned int index;
		m_work_list.get(idx, index);

		if (index < m_work_items.size()) {
			WorkItem *item = nullptr;
			getAt(index, &item);
			now = offsetTime();
			elapsed = now - item->m_queue_time;
			//DF_LOG_DEBUG("now = %lu elapsed = %lu delay = %luusec\n", now, elapsed, item.m_delay_usec);

			if (elapsed >= item->m_delay_usec) {

				DF_LOG_DEBUG("HRTWorkQueue::process do work (%p) (%u)", item, item->m_delay_usec);
				item->updateStats(now);

				// reschedule work
				item->m_queue_time += item->m_delay_usec;
				item->m_in_use = true;

				void *tmpptr = item->m_arg;
				m_lock.unlock();
				item->m_callback(tmpptr);
				m_lock.lock();
			}

			// Get next scheduling time
			uint64_t cur_next = item->m_queue_time + item->m_delay_usec;

			if (cur_next < next) {
				next = cur_next;
			}

			idx = m_work_list.next(idx);
		}


		m_lock.unlock();
	}
}

/*************************************************************************
  WorkMgr
*************************************************************************/
int WorkMgr::initialize()
{
	DF_LOG_DEBUG("WorkMgr::initialize");

	if (m_initialized) {
		DF_LOG_ERR("WorkMgr already initialized");
		return -1;
	}

	return 0;
}

void WorkMgr::finalize()
{
	DF_LOG_DEBUG("WorkMgr::finalize");

	if (!m_initialized) {
		DF_LOG_ERR("WorkMgr not initialized, cannot finalize");
		return;
	}

	WorkItems::finalize();
}

void WorkMgr::getWorkHandle(WorkCallback cb, void *arg, uint32_t delay_usec, WorkHandle &wh)
{
	unsigned int handle;

	int ret = WorkItems::getIndex(cb, arg, delay_usec, handle);
	if (ret == 0) {
		wh.m_errno = 0;
		wh.m_handle = (int)handle;
	} else {
		wh.m_errno = ret;
		wh.m_handle = -1;
	}
}

int WorkItems::getIndex(WorkCallback cb, void *arg, uint32_t delay_usec, unsigned int &index)
{
	WorkItems &inst = instance();

	inst.m_lock.lock();
	int ret = inst._getIndex(cb, arg, delay_usec, index);
	inst.m_lock.unlock();
	return ret;
}

int WorkItems::_getIndex(WorkCallback cb, void *arg, uint32_t delay_usec, unsigned int &index)
{
	int ret;

	// unschedule work and erase the handle if handle exists
	if (_isValidIndex(index)) {
		_unschedule(index);
	} else {
		// find an available WorkItem
		unsigned int i = 0;
		DFPointerList::Index idx = nullptr;
		idx = m_work_items.next(idx);

		while (idx != nullptr) {
			WorkItem *wi = reinterpret_cast<WorkItem *>(m_work_items.get(idx));

			if (!wi->m_in_use) {
				index = i;
				break;
			}

			++i;
			idx = m_work_items.next(idx);
		}

		// If no free WorkItems, add one to the end
		if (!_isValidIndex(index)) {
			m_work_items.pushBack(new WorkItem());
			index = m_work_items.size() - 1;
		}
	}

	if (_isValidIndex(index)) {
		// Re-use the WorkItem
		WorkItem *item;
		getAt(index, &item);

		item->set(cb, arg, delay_usec);
		ret = 0;
	} else {
		ret = EBADF;
	}

	return ret;
}

void WorkMgr::releaseWorkHandle(WorkHandle &wh)
{
	DF_LOG_DEBUG("WorkMgr::releaseWorkHandle");

	if (wh.m_handle == -1) {
		wh.m_errno = EBADF;
	}

	if (isValidHandle(wh)) {
		WorkItems::unschedule(wh.m_handle);
		wh.m_errno = 0;

	} else {
		wh.m_errno = EBADF;
	}
	wh.m_handle = -1;
}

int WorkMgr::schedule(DriverFramework::WorkHandle &wh)
{
	int ret = WorkItems::schedule(wh.m_handle);
	if (ret != 0) {
		wh.m_errno = ret;
	}
	return (ret == 0) ? 0 : -1;
}

int WorkItems::schedule(unsigned int index)
{
	WorkItems &inst = instance();

	inst.m_lock.lock();
	int ret = inst._schedule(index);
	inst.m_lock.unlock();
	return ret;
}

int WorkItems::_schedule(unsigned int index)
{
	DF_LOG_DEBUG("WorkItems::_schedule");

	int ret = 0;

	if (_isValidIndex(index)) {
		WorkItem *item;
		
		if (getAt(index, &item)) {
			if (item->m_in_use) {
				DF_LOG_ERR("WorkMgr::schedule can't schedule a handle that's in use");
				ret = EBUSY;

			} else {
				DF_LOG_DEBUG("WorkMgr::schedule - do schedule");
				item->m_queue_time = offsetTime();
				item->m_in_use = true;
				m_work_list.pushBack(index);
				//HRTWorkQueue::instance()->scheduleWorkItem(wh);
			}

		} else {
			DF_LOG_ERR("couldn't find handle to schedule");
			ret = EBADF;
		}
	} else {
		ret = EBADF;
	}

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
