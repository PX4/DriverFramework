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
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include "DevObj.hpp"
#include "DevIOCTL.h"

using namespace DriverFramework;

DevObj::DevObj(const char *name, const char *dev_path, const char *dev_class_path, DeviceBusType bus_type,
	       unsigned int sample_interval_usecs) :
	m_name(nullptr),
	m_dev_path(nullptr),
	m_dev_class_path(nullptr),
	m_dev_instance_path(nullptr),
	m_sample_interval_usecs(sample_interval_usecs),
	m_id {},
	m_work_handle {},
	m_pub_blocked(false),
	m_driver_instance(-1),
	m_refcount(0)
{
	m_id.dev_id_s.bus = 0;
	m_id.dev_id_s.address = 0;
	m_id.dev_id_s.devtype = bus_type;

	// dev_class_path might be nullptr
	if (dev_class_path) {
		m_dev_class_path = strdup(dev_class_path);
	}

	if (name) {
		m_name = strdup(name);

	} else {
		DF_LOG_ERR("ERROR: no name provided to DevObj::DevObj");
	}

	if (dev_path) {
		m_dev_path = strdup(dev_path);

	} else {
		DF_LOG_ERR("ERROR: no dev_path provided to DevObj::DevObj");
	}
}

DevObj::~DevObj()
{
	m_handle_lock.lock();
	DFPointerList::Index idx = nullptr;
	idx = m_handles.next(idx);

	while (idx != nullptr) {
		DevHandle *h = reinterpret_cast<DevHandle *>(m_handles.get(idx));

		if (h->isValid()) {
			m_handle_lock.unlock();
			DevMgr::releaseHandle(*h);
			m_handle_lock.lock();

		} else {
			idx = m_handles.erase(idx);
		}

		// m_handles may have been modified by DevMgr::releaseHandle()
		idx = nullptr;
		idx = m_handles.next(idx);
	}

	m_handle_lock.unlock();

	if (isRegistered()) {
		DevMgr::unregisterDriver(this);
		m_driver_instance = -1;
	}

	if (m_name) {
		free((void *)m_name);
	}

	if (m_dev_path) {
		free((void *)m_dev_path);
	}

	if (m_dev_class_path) {
		free((void *)m_dev_class_path);
	}

	if (m_dev_instance_path) {
		free((void *)m_dev_instance_path);
	}
}

int DevObj::init()
{
	DF_LOG_DEBUG("DevObj::init %s", m_name);

	if (!isRegistered()) {
		DF_LOG_DEBUG("DevObj::init registering %s", m_name);
		int ret = DevMgr::registerDriver(this);

		if (ret < 0) {
			DF_LOG_DEBUG("DevObj::init register failed %s", m_name);
			return ret;
		}

		m_driver_instance = ret;

	} else {
		DF_LOG_DEBUG("DevObj::init already registered %s", m_name);
	}

	return 0;
}

int DevObj::start()
{
	DF_LOG_DEBUG("DevObj::start %s", m_name);

	if (m_driver_instance < 0) {
		return -1;
	}

	if (m_work_handle.isValid()) {
		DF_LOG_ERR("Err: %s was already started", m_name);
		return -2;
	}

	// Can't start if no interval specified
	if (m_sample_interval_usecs == 0) {
		return -3;

	} else {
		do {
			WorkMgr::getWorkHandle(measure, this, m_sample_interval_usecs, m_work_handle);

			if (!m_work_handle.isValid()) {
				return -m_work_handle.getError();
			}

			DF_LOG_DEBUG("DevObj::start schedule %s", m_name);
			WorkMgr::schedule(m_work_handle);

			//if someone else calls getWorkHandle at the same time, both will return the same handle,
			//and one of the schedule() calls will then fail with EBUSY. In that case just retry.
		} while (m_work_handle.getError() == EBUSY);
	}

	return -m_work_handle.getError();
}

int DevObj::stop()
{
	DF_LOG_DEBUG("DevObj::stop %s", m_name);

	if (m_work_handle.isValid()) {
		WorkMgr::releaseWorkHandle(m_work_handle);

	} else {
		// Driver wasn't running
		return -1;
	}

	return 0;
}

int DevObj::devIOCTL(unsigned long request, unsigned long arg)
{
	DF_LOG_DEBUG("DevObj::ioctl %s", m_name);
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
	DF_LOG_DEBUG("DevObj::devRead %s", m_name);
	return -1;
}

ssize_t DevObj::devWrite(const void *buf, size_t count)
{
	DF_LOG_DEBUG("DevObj::devWrite %s", m_name);
	return -1;
}

void DevObj::measure(void *arg)
{
	reinterpret_cast<DevObj *>(arg)->_measure();
}

// Return -1 on failure, otherwise 0
int DevObj::addHandle(DevHandle &h)
{
	DF_LOG_DEBUG("DevObj::addHandle (%p)", &h);
	int ret = 0;
	m_handle_lock.lock();

	if (!isRegistered()) {
		ret = init();

		if (ret < 0) {
			DF_LOG_DEBUG("DevObj::addHandle init failed (%p)", &h);
			goto exit;
		}
	}

	if (m_handles.size() == 0) {
		// Start the driver if its not running
		if (!m_work_handle.isValid() && m_sample_interval_usecs > 0) {
			ret = start();

			if (ret < 0) {
				DF_LOG_DEBUG("DevObj::addHandle start failed (%p)", &h);
			}
		}
	}

	if (!m_handles.pushBack(&h)) {
		DF_LOG_INFO("DevObj::addHandle failed memory allocation");
		ret = -1;
	}

exit:
	m_handle_lock.unlock();
	DF_LOG_DEBUG("DevObj::addHandle end (%p)", &h);
	return (ret < 0) ? -1 : 0;
}

// Return -1 on failure, otherwise handle count
int DevObj::removeHandle(DevHandle &h)
{
	DF_LOG_DEBUG("DevObj::removeHandle (%p)", &h);
	m_handle_lock.lock();
	DFPointerList::Index idx = nullptr;
	idx = m_handles.next(idx);

	while (idx != nullptr) {
		DevHandle *list_h = reinterpret_cast<DevHandle *>(m_handles.get(idx));

		if (list_h == &h) {
			idx = m_handles.erase(idx);

			//Do not stop driver even when the last handle is closed
			break;
		}

		idx = m_handles.next(idx);
	}

	m_handle_lock.unlock();
	return m_handles.size();
}

void DevObj::updateNotify()
{
	DF_LOG_DEBUG("DevObj::updateNotify");
	DevMgr::updateNotify(*this);
}

void DevObj::setSampleInterval(unsigned int sample_interval_usecs)
{
	m_sample_interval_usecs = sample_interval_usecs;

	// If running
	if (m_work_handle.isValid()) {
		if (m_sample_interval_usecs == 0) {
			stop();
			return;
		}

		// Reschedule if non-zero and initialized
		if (m_sample_interval_usecs != 0) {
			WorkMgr::releaseWorkHandle(m_work_handle);

			do {
				WorkMgr::getWorkHandle(measure, this, m_sample_interval_usecs, m_work_handle);

				if (!m_work_handle.isValid()) {
					return;
				}

				WorkMgr::schedule(m_work_handle);

				//if someone else calls getWorkHandle at the same time, both will return the same handle,
				//and one of the schedule() calls will then fail with EBUSY. In that case just retry.
			} while (m_work_handle.getError() == EBUSY);
		}
	}
}
