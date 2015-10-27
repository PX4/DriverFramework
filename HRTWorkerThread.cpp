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
#include <pthread.h>
#include <time.h>
#include "HRTWorkerThread.hpp"

// Static Variables

static uint64_t g_timestart = 0;
static pthread_t g_tid;

static HRTWorkerThread *g_instance = NULL;

static pthread_mutex_t g_hrt_lock = PTHREAD_RECURSIVE_MUTEX_INITIALIZER_NP;
static pthread_cond_t g_reschedule_cond = PTHREAD_COND_INITIALIZER;

// Static Functions

static uint64_t TSToABSTime(struct timespec *ts)
{
        uint64_t result;

        result = (uint64_t)(ts->tv_sec) * 1000000;
        result += ts->tv_nsec / 1000;

        return result;
}

// Class Methods

void *HRTWorkerThread::process_trampoline(void *arg)
{
	std::cout << "HRTWorkerThread::process_trampoline\n";
	if (g_instance) {
		g_instance->process();
	}
	return NULL;
}

uint64_t HRTAbsoluteTime(void)
{
        struct timespec ts;

        if (!g_timestart) {
                clock_gettime(CLOCK_MONOTONIC, &ts);
                g_timestart = TSToABSTime(&ts);
        }

        clock_gettime(CLOCK_MONOTONIC, &ts);
        return TSToABSTime(&ts) - g_timestart;
}

void WorkItemStats::update(unsigned int cur_usec)
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

void WorkItemStats::reset() 
{
	m_last = ~(uint64_t)0;
	m_min = ~(uint64_t)0;
	m_max = 0;
	m_total = 0;
	m_count = 0;
}

HRTWorkerThread *HRTWorkerThread::instance(void)
{
	if (g_instance)
		return g_instance;

	g_instance = new HRTWorkerThread();
	return g_instance;
}

int HRTWorkerThread::init(void)
{
	std::cout << "HRTWorkerThread::init\n";
	// Create a lock for handling the work queue
	if (pthread_mutex_init(&g_hrt_lock, NULL) != 0)
	{
        	return 1;
	}

	// Create high priority worker thread
	int ret = pthread_create(&g_tid, NULL, process_trampoline, NULL);
	return ret;
}

void HRTWorkerThread::cleanup(void)
{
	std::cout << "HRTWorkerThread::cleanup\n";
	pthread_join(g_tid, NULL);
	pthread_mutex_destroy(&g_hrt_lock);
}

void HRTWorkerThread::scheduleWorkItem(std::shared_ptr<WorkItem> item)
{
	std::cout << "HRTWorkerThread::scheduleWorkItem\n";
	hrtLock();
	std::cout << "Add WorkItem\n";
	m_work.push_back(item);
	item->m_queue_time = HRTAbsoluteTime();
	std::cout << "Signal\n";
	pthread_cond_signal(&g_reschedule_cond);
	hrtUnlock();
}

void HRTWorkerThread::process(void)
{
	std::cout << "HRTWorkerThread::process\n";
	std::list<std::shared_ptr<WorkItem>>::iterator work_itr;
	uint64_t delta;
	uint64_t next;
	uint64_t elapsed;
	uint64_t remaining;

	for(;;) {
		hrtLock();

		// Wake up every 10 sec if nothing scheduled
		next = 10000000;
		work_itr  = m_work.begin();

		std::cout << "Before While\n";
		while (work_itr != m_work.end()) {
			elapsed = HRTAbsoluteTime() - work_itr->get()->m_queue_time;
			std::cout << "Process work item elapsed = " << elapsed << "\n";

			if (elapsed >= work_itr->get()->m_delay) {
				std::cout << "Elapsed\n";
				std::shared_ptr<WorkItem> dequeuedWork = *work_itr;
				work_itr = m_work.erase(work_itr);

				// Mutex is recursive so the WorkItem can be rescheduled
				// while the lock is held
				dequeuedWork->m_callback(dequeuedWork->m_arg);

			} else {
				std::cout << "Not expired (delay = " << work_itr->get()->m_delay << "\n";
				remaining = work_itr->get()->m_delay - elapsed;
				if (remaining < next) {
					next = remaining;
				}

				// try the next in the list
				++work_itr;
			}
		}

		std::cout << "Next = " << next << "\n";

		// pthread_cond_timedwait uses absolute time
		timespec ts;
        	clock_gettime(CLOCK_REALTIME, &ts);
		uint64_t nanosecs = (ts.tv_nsec + ((next % 1000000)*1000));
		uint32_t secs = nanosecs / 1000000000;
		ts.tv_sec += next/1000000 + secs; 
		ts.tv_nsec = nanosecs % 1000000000;
		
		// Wait until next expiry or until a new item is rescheduled
		int rc = pthread_cond_timedwait(&g_reschedule_cond, &g_hrt_lock, &ts);
		std::cout << " rc = " << rc << "\n";
		hrtUnlock();
	}
}

void HRTWorkerThread::hrtLock()
{
	std::cout << "lock\n";
	pthread_mutex_lock(&g_hrt_lock);
}

void HRTWorkerThread::hrtUnlock()
{
	std::cout << "unlock\n";
	pthread_mutex_unlock(&g_hrt_lock);
}

