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
#include <iostream>
#include <list>
#include <map>
#include <pthread.h>
#include "DriverFramework.hpp"


namespace DriverFramework {

namespace WorkItemMgr {

int initialize(void);
void finalize(void);

};
};

using namespace DriverFramework;
//-----------------------------------------------------------------------
// Types
//-----------------------------------------------------------------------
class WorkItem
{
public:
	WorkItem(workCallback callback, void *arg, uint32_t delay, WorkHandle handle) : 
		m_arg(arg),
		m_queue_time(0),
		m_callback(callback),
		m_delay(delay),
		m_handle(handle)
	{
		resetStats();
	}
	~WorkItem() {}

	void schedule();

	void updateStats(unsigned int cur_usec);
	void resetStats();

	void *		m_arg;
	uint64_t	m_queue_time;
	workCallback	m_callback;
	uint32_t	m_delay;
	WorkHandle	m_handle;

	// statistics
	unsigned long m_last;
	unsigned long m_min;
	unsigned long m_max;
	unsigned long m_total;
	unsigned long m_count;
};

class HRTWorkQueue
{
public:
	static HRTWorkQueue *instance(void);

	static int initialize(void);
	static void finalize(void);

	void scheduleWorkItem(WorkItem &item);

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

	std::list<WorkItem>	m_work;

	bool m_enable_stats = false;
	bool m_exit_requested = false;

	static HRTWorkQueue *m_instance;
};

//-----------------------------------------------------------------------
// Static Variables
//-----------------------------------------------------------------------

static uint64_t g_timestart = 0;
static pthread_t g_tid;

static pthread_mutex_t g_framework_exit = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t g_hrt_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t g_reschedule_cond = PTHREAD_COND_INITIALIZER;
static pthread_cond_t g_framework_cond = PTHREAD_COND_INITIALIZER;

static std::map<WorkHandle, WorkItem> *g_work_items = nullptr;

//-----------------------------------------------------------------------
// Static Functions
//-----------------------------------------------------------------------

static uint64_t TSToABSTime(struct timespec *ts)
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
        struct timespec ts;

       	clock_gettime(CLOCK_REALTIME, &ts);
	if (!g_timestart) {
		g_timestart = TSToABSTime(&ts);
	}

	// Time is in microseconds
        return TSToABSTime(&ts) - g_timestart;
}

timespec DriverFramework::offsetTimeToAbsoluteTime(uint64_t offset_time)
{
	uint64_t abs_time = offset_time + g_timestart;
        struct timespec ts;
	ts.tv_sec = abs_time / 1000000;
	ts.tv_nsec = (abs_time % 1000000) * 1000;

	return ts;
}

//-----------------------------------------------------------------------
// Class Methods
//-----------------------------------------------------------------------

/*************************************************************************
  DriverFramework
*************************************************************************/
void DriverFramework::shutdown()
{
	std::cout << "DriverFramework::shutdown\n";
	// Stop the HRT queue thread
	HRTWorkQueue *wq = HRTWorkQueue::instance();
	if (wq) {
		wq->shutdown();
	}

	// Free the resources
	HRTWorkQueue::finalize();

	// allow Framework to exit
	pthread_mutex_lock(&g_framework_exit);
	pthread_cond_signal(&g_framework_cond);
	pthread_mutex_unlock(&g_framework_exit);
}

int DriverFramework::initialize()
{
	std::cout << "DriverFramework::initialize\n";
	int ret = HRTWorkQueue::initialize();
	if (ret < 0) {
		return ret;
	}
#if 0
	ret = DriverFramework::DriverMgr::initialize();
	if (ret < 0) {
		return ret-10;
	}
#endif
	ret = DriverFramework::WorkItemMgr::initialize();
	if (ret < 0) {
		return ret-20;
	}
	return 0;
}

void DriverFramework::waitForShutdown()
{
	std::cout << "DriverFramework::waitForShutdown\n";
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
	unsigned int delay = (m_last == ~0x0UL) ? cur_usec : (cur_usec - m_last);

	if (delay < m_min) {
		m_min = delay;
	}
	if (delay > m_max) {
		m_max = delay;
	}

	m_total += delay;
	m_count += 1;
	m_last = cur_usec;
}

void WorkItem::resetStats() 
{
	m_last = ~(uint64_t)0;
	m_min = ~(uint64_t)0;
	m_max = 0;
	m_total = 0;
	m_count = 0;
}

/*************************************************************************
  HRTWorkQueue
*************************************************************************/
HRTWorkQueue *HRTWorkQueue::m_instance = NULL;

void *HRTWorkQueue::process_trampoline(void *arg)
{
	std::cout << "HRTWorkQueue::process_trampoline\n";
	if (m_instance) {
		m_instance->process();
	}
	return NULL;
}

HRTWorkQueue *HRTWorkQueue::instance(void)
{
	return m_instance;
}

int HRTWorkQueue::initialize(void)
{
	m_instance = new HRTWorkQueue();

	if (m_instance == nullptr) {
		return -2;
	}

	std::cout << "HRTWorkQueue::initialize\n";
	// Create a lock for handling the work queue
	if (pthread_mutex_init(&g_hrt_lock, NULL) != 0)
	{
        	return -1;
	}

	// Create high priority worker thread
	int ret = pthread_create(&g_tid, NULL, process_trampoline, NULL);
	return ret;
}

void HRTWorkQueue::finalize(void)
{
	std::cout << "HRTWorkQueue::cleanup\n";
	pthread_join(g_tid, NULL);
	HRTWorkQueue *wq = HRTWorkQueue::instance();
	if (wq) {
		wq->clearAll();
		WorkItemMgr::finalize();

		pthread_mutex_destroy(&g_hrt_lock);

		delete wq;
		wq = nullptr;
	}
}

void HRTWorkQueue::scheduleWorkItem(WorkItem &item)
{
	std::cout << "HRTWorkQueue::scheduleWorkItem\n";
	hrtLock();
	std::cout << "Add WorkItem\n";
	m_work.push_back(item);
	item.m_queue_time = offsetTime();
	std::cout << "Signal\n";
	pthread_cond_signal(&g_reschedule_cond);
	hrtUnlock();
}

void HRTWorkQueue::clearAll()
{
	std::cout << "HRTWorkQueue::clearAll\n";
	hrtLock();
	m_work.clear();
	hrtUnlock();
}

void HRTWorkQueue::shutdown(void)
{
	hrtLock();
	m_exit_requested = true;
	pthread_cond_signal(&g_reschedule_cond);
	hrtUnlock();
}

void HRTWorkQueue::process(void)
{
	std::cout << "HRTWorkQueue::process\n";
	std::list<WorkItem>::iterator work_itr;
	uint64_t delta;
	uint64_t next;
	uint64_t elapsed;
	uint64_t remaining;
	timespec ts;
	timespec ts2;
	uint64_t now;

	for(;!m_exit_requested;) {
		hrtLock();

		// Wake up every 10 sec if nothing scheduled
		next = 10000000;
		work_itr = m_work.begin();

		std::cout << "Before While --- \n";
		now = offsetTime();
		while (work_itr != m_work.end()) {
			now = offsetTime();
			elapsed = now - work_itr->m_queue_time;
			std::cout << "Process work item elapsed = " << elapsed << " " << work_itr->m_delay << "\n";

			if (elapsed >= work_itr->m_delay) {
				std::cout << "Elapsed\n";
				WorkItem &dequeuedWork = *work_itr;

				// Mutex is recursive so the WorkItem can be rescheduled
				// while the lock is held
				hrtUnlock();
				dequeuedWork.m_callback(dequeuedWork.m_arg, dequeuedWork.m_handle);
				hrtLock();

				work_itr = m_work.erase(work_itr);

			} else {
				std::cout << "Not expired (delay = " << work_itr->m_delay << "\n";
				remaining = work_itr->m_delay - elapsed;
				if (remaining < next) {
					next = remaining;
				}

				// try the next in the list
				++work_itr;
			}
		}

		std::cout << "Next = " << next << "\n";

		// pthread_cond_timedwait uses absolute time
		ts = offsetTimeToAbsoluteTime(now+next);
		
		// Wait until next expiry or until a new item is rescheduled
		std::cout << "Waiting...\n";
		int rc = pthread_cond_timedwait(&g_reschedule_cond, &g_hrt_lock, &ts);
		std::cout << " rc = " << rc << "\n";
		hrtUnlock();
	}
}

void HRTWorkQueue::hrtLock()
{
	std::cout << "lock\n";
	pthread_mutex_lock(&g_hrt_lock);
}

void HRTWorkQueue::hrtUnlock()
{
	std::cout << "unlock\n";
	pthread_mutex_unlock(&g_hrt_lock);
}

/*************************************************************************
  WorkItemMgr
*************************************************************************/
int WorkItemMgr::initialize()
{
	g_work_items = new std::map<WorkHandle,WorkItem>;
	return 0;
}

void WorkItemMgr::finalize()
{
	while(g_work_items->begin() != g_work_items->end())
	{
		std::map<WorkHandle,WorkItem>::iterator it = g_work_items->begin();
		
		// Remove the element from the map
		g_work_items->erase(it);
	}
	delete g_work_items;
	g_work_items = nullptr;
}

WorkHandle WorkItemMgr::create(workCallback cb, void *arg, uint32_t delay)
{
	std::cout << "WorkItemMgr::create\n";
	static WorkHandle i=1000;
	std::map<WorkHandle,WorkItem>::iterator it = g_work_items->begin();
	++i;
  	g_work_items->insert (it, std::pair<WorkHandle,WorkItem>(i, WorkItem(cb, arg, delay, i)));
	std::cout << "Handle = " << i << "\n";
	return i;
}

void WorkItemMgr::destroy(WorkHandle handle)
{
	// remove from map, then from work queue, then delete
	std::map<WorkHandle,WorkItem>::iterator it = g_work_items->find(handle);
	if (it != g_work_items->end()) {
		g_work_items->erase(it);
	}
}

bool WorkItemMgr::schedule(WorkHandle handle)
{
	std::cout << "WorkItemMgr::schedule " << handle << "\n";
	std::map<WorkHandle,WorkItem>::iterator it = g_work_items->find(handle);
	bool ret = it != g_work_items->end();
	if (ret) {
		HRTWorkQueue::instance()->scheduleWorkItem(it->second);
	}
	return ret;
}
