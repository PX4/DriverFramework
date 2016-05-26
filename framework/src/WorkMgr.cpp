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
	return ((h.m_handle >= 0) && WorkItems::isValidIndex(h.m_handle));
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

	m_initialized = true;

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
	// Use -1 to flag that we don't know the index, otherwise we pass undefined.
	int handle = -1;

	int ret = WorkItems::getIndex(cb, arg, delay_usec, handle);

	if (ret == 0) {
		wh.m_errno = 0;
		wh.m_handle = (int)handle;

	} else {
		wh.m_errno = ret;
		wh.m_handle = -1;
	}
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
