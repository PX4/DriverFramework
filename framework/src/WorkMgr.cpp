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
#include "WorkItems.hpp"

using namespace DriverFramework;

bool WorkMgr::m_initialized = false;

//-----------------------------------------------------------------------
// Static Functions
//-----------------------------------------------------------------------

// TODO FIXME: this seems conflicting with WorkHandle::isValid()
bool WorkMgr::isValidHandle(const WorkHandle &h)
{
	return ((h.m_handle >= 0) && WorkItems::isValidIndex((unsigned int)h.m_handle));
}

//-----------------------------------------------------------------------
// Class Methods
//-----------------------------------------------------------------------

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

void WorkMgr::setError(WorkHandle &h, int error)
{
	h.m_errno = error;
}

WorkHandle::~WorkHandle()
{
	WorkMgr::releaseWorkHandle(*this);
}
