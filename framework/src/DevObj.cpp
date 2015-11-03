/**********************************************************************
* Copyright (c) 2012 Lorenz Meier
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
#include "DevObj.hpp"

using namespace DriverFramework;

DevObj::DevObj(const char *name, const char *dev_base_path, DeviceBusType bus_type, unsigned int sample_interval) :
	m_name(name),
	m_dev_base_path(dev_base_path),
	m_sample_interval(sample_interval),
	m_driver_instance(-1),
	m_refcount(0)
{
	m_id.dev_id_s.bus = 0;
	m_id.dev_id_s.address = 0;
	m_id.dev_id_s.devtype = bus_type;
}

int DevObj::start(void)
{
	if (m_driver_instance < 0) {
		int ret = DevMgr::registerDriver(this);
		if (ret < 0) {
			return ret;
		}
		m_driver_instance = ret;
	}
	if (m_sample_interval && !m_work_handle) {
		m_work_handle = WorkMgr::create(measure, this, m_sample_interval);
		WorkMgr::schedule(m_work_handle);
	}
	return 0;
}

int DevObj::stop(void) {
	if (m_work_handle) {
		WorkMgr::destroy(m_work_handle);
		m_work_handle=0;
		DevMgr::unregisterDriver(this);
	}
	return 0;
}

DevObj::~DevObj() 
{
	std::list<DevHandle *>::iterator it = m_handles.begin();
	while (it != m_handles.end()) {
		if ((*it)->isValid()) {
			DevMgr::releaseHandle(**m_handles.begin());
		}
		else {
			m_handles.erase(it);
		}
		it = m_handles.begin();
	}

	if (isRegistered()) {
		DevMgr::unregisterDriver(this);
	}
}

int DevObj::devIOCTL(unsigned long request, void *arg)
{
	return -1;
}

ssize_t DevObj::devRead(void *buf, size_t count)
{
	return -1;
}

ssize_t DevObj::devWrite(void *buf, size_t count)
{
	return -1;
}

void DevObj::measure(void *arg, const WorkHandle wh)
{
	// Reschedule callback
	WorkMgr::schedule(wh);

	reinterpret_cast<DevObj *>(arg)->_measure();
}

// Return -1 on failure, otherwise recount
int DevObj::addHandle(DevHandle &h)
{
	if (m_handles.size() == 0) {
		int ret = start();
		if (ret < 0) {
			return -1;
		}
	}
	m_handles.push_back(&h);
	return m_handles.size();
}

// Return -1 on failure, otherwise recount
int DevObj::removeHandle(DevHandle &h)
{
	std::list<DevHandle *>::iterator it = m_handles.begin();
	while (it != m_handles.end()) {
		if (*it == &h) {
			it = m_handles.erase(it);
			if (it == m_handles.end()) {
				stop();
			}
		}
		++it;
	}
	return m_handles.size();
}

void DevObj::updateNotify()
{
	DevMgr::updateNotify(*this);
}
