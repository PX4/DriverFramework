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
*  * Neither the name of PX4 Project nor the names of its
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
#include "DevIOCTL.h"

using namespace DriverFramework;

DevObj::DevObj(const char *name, const char *dev_path, const char *dev_class_path, DeviceBusType bus_type, unsigned int sample_interval_usecs) :
	m_name(name),
	m_dev_path(dev_path),
	m_sample_interval_usecs(sample_interval_usecs),
	m_pub_blocked(false),
	m_driver_instance(-1),
	m_refcount(0)
{
	m_id.dev_id_s.bus = 0;
	m_id.dev_id_s.address = 0;
	m_id.dev_id_s.devtype = bus_type;

	// dev_class_path might be nullptr
	if (dev_class_path) {
		m_dev_class_path.assign(dev_class_path);
	}
}

int DevObj::init(void)
{
	DF_LOG_INFO("DevObj::init %s", m_name.c_str());
	if (!isRegistered()) {
		DF_LOG_DEBUG("DevObj::init registering %s", m_name.c_str());
		int ret = DevMgr::registerDriver(this);
		if (ret < 0) {
			DF_LOG_DEBUG("DevObj::init register failed %s", m_name.c_str());
			return ret;
		}
		m_driver_instance = ret;
	} else {
		DF_LOG_DEBUG("DevObj::init already registered %s", m_name.c_str());
	}
	return 0;
}

int DevObj::start(void)
{
	DF_LOG_INFO("DevObj::start %s", m_name.c_str());

	if (m_driver_instance < 0) {
		return -1;
	}

	if (m_work_handle.isValid()) {
		DF_LOG_INFO("Err: %s was already started", m_name.c_str());
		return -2;
	}

	// Only schedule periodic work if an interval was specfied
	if (m_sample_interval_usecs) {
		WorkMgr::getWorkHandle(measure, this, m_sample_interval_usecs, m_work_handle);
		if (m_work_handle.isValid()) {
			DF_LOG_DEBUG("DevObj::start schedule %s", m_name.c_str());
			WorkMgr::schedule(m_work_handle);
		}
		else {
			return -3;
		}
	}
	return 0;
}

int DevObj::stop(void) {
	DF_LOG_INFO("DevObj::stop %s", m_name.c_str());
	if (m_work_handle.isValid()) {
		WorkMgr::releaseWorkHandle(m_work_handle);
	}
	return 0;
}

DevObj::~DevObj() 
{
	m_handle_lock.lock();
	std::list<DevHandle *>::iterator it = m_handles.begin();
	while (it != m_handles.end()) {
		if ((*it)->isValid()) {
			m_handle_lock.unlock();
			DevMgr::releaseHandle(**m_handles.begin());
			m_handle_lock.lock();
		}
		else {
			m_handles.erase(it);
		}
		// m_handles may have been modified by DevMgr::releaseHandle()
		it = m_handles.begin();
	}
	m_handle_lock.unlock();

	if (isRegistered()) {
		DevMgr::unregisterDriver(this);
	}
}

int DevObj::devIOCTL(unsigned long request, unsigned long arg)
{
	int ret = -1;

	switch (request) {

	// If the driver is using a pub-sub model, enable/disable publish
        case DEVIOCSPUBBLOCK:
                m_pub_blocked = ((void *)arg != nullptr);
                ret = 0;
                break;

	// If the driver is using a pub-sub model, get publish state
        case DEVIOCGPUBBLOCK:
		ret = m_pub_blocked;
                break;

	// Get the device ID
	case DEVIOCGDEVICEID:
                return (int)m_id.dev_id;

	default:
                break;
        }

	return ret;
}

ssize_t DevObj::devRead(void *buf, size_t count)
{
	return -1;
}

ssize_t DevObj::devWrite(const void *buf, size_t count)
{
	return -1;
}

void DevObj::measure(void *arg)
{
	reinterpret_cast<DevObj *>(arg)->_measure();
}

// Return -1 on failure, otherwise recount
int DevObj::addHandle(DevHandle &h)
{
	DF_LOG_DEBUG("DevObj::addHandle (%p)", &h);
	int ret = 0;
	m_handle_lock.lock();
	if (m_handles.size() == 0) {

		// Start the driver if its not running
		if (!m_work_handle.isValid()) {
			ret = start();
			if (ret < 0) {
				DF_LOG_DEBUG("DevObj::addHandle start failed (%p)", &h);
			}
		}
	}
	m_handles.push_back(&h);
	m_handle_lock.unlock();
	DF_LOG_DEBUG("DevObj::addHandle end (%p)", &h);
	return (ret < 0) ? -1 : m_handles.size();
}

// Return -1 on failure, otherwise recount
int DevObj::removeHandle(DevHandle &h)
{
	DF_LOG_DEBUG("DevObj::removeHandle (%p)", &h);
	int ret = 0;
	m_handle_lock.lock();
	std::list<DevHandle *>::iterator it = m_handles.begin();
	while (it != m_handles.end()) {
		if (*it == &h) {
			it = m_handles.erase(it);
			if (it == m_handles.end()) {

				// TODO - do we need special handling if that last handle to a device is closed?

				if (ret != 0) {
					ret = -1;
				}
			}
		}
		++it;
	}
	m_handle_lock.unlock();
	return (ret == 0) ? m_handles.size() : ret;
}

void DevObj::updateNotify()
{
	DF_LOG_DEBUG("DevObj::updateNotify");
	DevMgr::updateNotify(*this);
}

void DevObj::setSampleInterval(unsigned int sample_interval_usecs)
{
	if (m_sample_interval_usecs != sample_interval_usecs) {
		WorkMgr::getWorkHandle(measure, this, m_sample_interval_usecs, m_work_handle);

		// If running, then reschedule
		if (m_sample_interval_usecs != 0 && m_driver_instance >= 0) {
			WorkMgr::schedule(m_work_handle);
		}
	}
}
