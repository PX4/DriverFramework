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
#if SHOW_STATS == 1
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


	if ((m_count % 100) == 99) {
		dumpStats();
	}

#endif
}

void WorkItems::WorkItem::resetStats()
{
#if SHOW_STATS == 1
	m_last = ~(unsigned long)0;
	m_min = ~(unsigned long)0;
	m_max = 0;
	m_total = 0;
	m_count = 0;
#endif
}

void WorkItems::WorkItem::dumpStats()
{
#if SHOW_STATS == 1
	DF_LOG_DEBUG("Stats for callback=%p: count=%lu, avg=%lu min=%lu max=%lu",
		     m_callback, m_count, m_total / m_count, m_min, m_max);
#endif
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

				// try to align scheduling time with already existing items. This reduces scheduling overhead
				// as we have to wake up less often. For that to work well (in the sense of low runtime jitter),
				// each callback must have a roughly constant execution time.
				// We search another item with the following priorities:
				// 1. item has equal sampling rate
				// 2. item has a sampling rate that is a multiple of wi's sampling rate
				// 3. wi has a sampling rate that is a multiple of item's sampling rate
				// 4. just pick an arbitrary wi
				DFPointerList::Index idx = nullptr;
				idx = m_work_items.next(idx);
				uint64_t queue_time_equal = 0, queue_time_multiple = 0, queue_time_divider = 0, queue_time_other = 0;

				while (idx != nullptr) {
					WorkItem *wi = reinterpret_cast<WorkItem *>(m_work_items.get(idx));

					if (wi->m_in_use) {
						if (wi->m_delay_usec == item->m_delay_usec) {
							queue_time_equal = wi->m_queue_time;

						} else if (item->m_delay_usec % wi->m_delay_usec == 0) {
							queue_time_multiple = wi->m_queue_time;

						} else if (wi->m_delay_usec % item->m_delay_usec == 0) {
							queue_time_divider = wi->m_queue_time;

						} else {
							queue_time_other = wi->m_queue_time;
						}
					}

					idx = m_work_items.next(idx);
				}

				uint64_t now = offsetTime();

				if (queue_time_equal) {
					item->m_queue_time = queue_time_equal;

				} else if (queue_time_multiple) {
					item->m_queue_time = queue_time_multiple;

				} else if (queue_time_divider) {
					item->m_queue_time = queue_time_divider;

				} else if (queue_time_other) {
					item->m_queue_time = queue_time_other;

				} else {
					item->m_queue_time = now;
				}

				// make sure next scheduling is in the future
				while (item->m_queue_time + item->m_delay_usec < now) {
					item->m_queue_time += item->m_delay_usec;
				}

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
	uint64_t now;
	uint32_t elapsed;
	uint32_t max_too_late_scheduled = 0;
	bool had_work = false;

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
			if (!item->m_in_use) {
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

				if (!had_work && elapsed - item->m_delay_usec > max_too_late_scheduled) {
					//only take the first into account, because we don't want to include the callback
					//execution time of the previous items
					max_too_late_scheduled = elapsed - item->m_delay_usec;
				}

				void *tmpptr = item->m_arg;
				WorkCallback cb = item->m_callback;
				m_lock.unlock();
				cb(tmpptr);
				had_work = true;
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


#if 0 //debug the scheduling adjustment
	static int no_work_counter = 0;

	if (had_work) {
		static uint32_t max_late_stat = max_too_late_scheduled;
		static uint64_t max_late_sum = 0;
		static int counter = 0;

		if (max_too_late_scheduled > max_late_stat) {
			max_late_stat = max_too_late_scheduled;
		}

		max_late_sum += max_too_late_scheduled;

		if (++counter == 200) {
			DF_LOG_ERR("max late= %3i us mean late=%3i us  no work=%i, cur_adj=%i",
				   (int)max_late_stat, (int)(max_late_sum / counter), no_work_counter, m_scheduling_adjustment);
			counter = 0;
			max_late_stat = 0;
			no_work_counter = 0;
			max_late_sum = 0;
		}

	} else {
		++no_work_counter;
	}

#endif

// disable scheduling adjustment on embedded platforms (tests showed worse performance on RPI & QuRT with this)
// see test results: https://github.com/PX4/DriverFramework/pull/155
#if defined(__DF_LINUX) && !defined(__DF_RPI) && !defined(__DF_BEBOP) && !defined(__DF_EDISON)

	if (had_work) {
		// Scheduling can have jitter, so adjust only by a fraction.
		// The chosen factors are a tradeoff between low-latency and CPU overhead
		m_scheduling_adjustment += max_too_late_scheduled / 5;

		if (m_scheduling_adjustment > 1e4) { //max to 10ms
			m_scheduling_adjustment = 1e4;
		}

	} else {
		// We woke up for nothing. Reduce the adjustment
		m_scheduling_adjustment = m_scheduling_adjustment * 90 / 100;
	}

	next -= m_scheduling_adjustment;
#endif

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

