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

#include <errno.h>
#include "DriverFramework.hpp"
#include "SyncObj.hpp"
#include "WorkItems.hpp"

using namespace DriverFramework;

bool WorkItems::isValidIndex(int index)
{
	WorkItems &inst = instance();

	inst.m_lock.lock();
	bool ret = inst._isValidIndex(index);
	inst.m_lock.unlock();
	return ret;
}

bool WorkItems::_isValidIndex(int index)
{
	return (index >= 0 && (index < (int)m_work_items.size()));
}

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

int WorkItems::schedule(int index)
{
	DF_LOG_DEBUG("WorkItems::schedule");
	WorkItems &inst = instance();

	inst.m_lock.lock();
	int ret = inst._schedule(index);
	inst.m_lock.unlock();
	return ret;
}

int WorkItems::_schedule(int index)
{
	DF_LOG_DEBUG("WorkItems::_schedule");

	int ret = 0;

	if (_isValidIndex(index)) {
		WorkItem *item = nullptr;

		if (getAt(index, &item)) {
			if (item->m_in_use) {
				DF_LOG_ERR("WorkMgr::schedule can't schedule a handle that's in use");
				ret = EBUSY;

			} else {
				DF_LOG_DEBUG("WorkMgr::schedule - do schedule");
				item->m_queue_time = offsetTime();
				item->m_in_use = true;
				m_work_list.pushBack(index);
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

void WorkItems::unschedule(int index)
{
	WorkItems &inst = instance();

	inst.m_lock.lock();
	inst._unschedule(index);
	inst.m_lock.unlock();
}

void WorkItems::_unschedule(int index)
{
	DFUIntList::Index idx = nullptr;
	idx = m_work_list.next(idx);

	while (idx != nullptr) {
		// If we find it in the list at the current idx, let's go ahead and delete it.
		unsigned int cur_index;

		if (m_work_list.get(idx, cur_index)) {

			if ((int)cur_index == index) {
				// remove unscheduled item
				WorkItem *item = nullptr;

				if (!getAt(index, &item)) {
					DF_LOG_ERR("HRTWorkQueue::unscheduleWorkItem - invalid index");

				} else {
					item->m_in_use = false;
					// We're only unscheduling one item, so we can bail out here.
					break;
				}
			}
		}

		idx = m_work_list.next(idx);
	}
}

void WorkItems::processExpiredWorkItems(uint64_t &next)
{
	DF_LOG_DEBUG("WorkItems::processExpiredWorkItems %" PRIu64 "", next);
	WorkItems &inst = instance();

	inst.m_lock.lock();
	inst._processExpiredWorkItems(next);
	inst.m_lock.unlock();
}

void WorkItems::_processExpiredWorkItems(uint64_t &next)
{
	DF_LOG_DEBUG("WorkItems::processExpiredWorkItems");
	uint64_t elapsed;
	uint64_t now;

	DFUIntList::Index idx = nullptr;
	idx = m_work_list.next(idx);

	while (g_run_status && g_run_status->check() && (idx != nullptr)) {
		DF_LOG_DEBUG("HRTWorkQueue::process work exists");
		unsigned int index;
		m_work_list.get(idx, index);


		if (index < m_work_items.size()) {
			WorkItem *item = nullptr;
			getAt(index, &item);
			DF_LOG_DEBUG("WorkList (%p) in use=%d delay=%u queue_time=%" PRIu64 , item, item->m_in_use, item->m_delay_usec,
				     item->m_queue_time);

			// Remove inactive work items from work list here to prevent use after free
			if (item->m_in_use == false) {
				// Remove the inactive work item from work list
				idx = m_work_list.erase(idx);
				continue;
			}

			now = offsetTime();
			elapsed = now - item->m_queue_time;
			//DF_LOG_DEBUG("now = %lu elapsed = %lu delay = %luusec\n", now, elapsed, item.m_delay_usec);

			if (elapsed >= item->m_delay_usec) {
				DF_LOG_DEBUG("WorkItems::processExpiredWorkItems  do work: (%p) (%u)", item, item->m_delay_usec);
				item->updateStats(now);

				// reschedule work
				item->m_queue_time += item->m_delay_usec;
				item->m_in_use = true;

				void *tmpptr = item->m_arg;
				WorkCallback cb = item->m_callback;
				m_lock.unlock();
				cb(tmpptr);
				m_lock.lock();
			}

			// Get next scheduling time
			uint64_t cur_next = item->m_queue_time + item->m_delay_usec;

			if (cur_next < next) {
				next = cur_next;
			}

			idx = m_work_list.next(idx);
		}
	}

	DF_LOG_DEBUG("Setting next=%" PRIu64, next);
}

int WorkItems::getIndex(WorkCallback cb, void *arg, uint32_t delay_usec, int &index)
{
	WorkItems &inst = instance();

	inst.m_lock.lock();
	int ret = inst._getIndex(cb, arg, delay_usec, index);
	inst.m_lock.unlock();
	return ret;
}

int WorkItems::_getIndex(WorkCallback cb, void *arg, uint32_t delay_usec, int &index)
{
	int ret;

	// unschedule work and erase the handle if handle exists
	if (_isValidIndex(index)) {
		_unschedule(index);

	} else {
		// find an available WorkItem
		unsigned i = 0;
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
		WorkItem *item = nullptr;
		getAt(index, &item);

		item->set(cb, arg, delay_usec);
		ret = 0;

	} else {
		ret = EBADF;
	}

	return ret;
}

