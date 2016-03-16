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
#include <string.h>
#include "DriverFramework.hpp"
#include "DFList.hpp"
#include "SyncObj.hpp"
#include "DevObj.hpp"
#include "DevMgr.hpp"

#include <stdlib.h>
#include <errno.h>

using namespace DriverFramework;

static SyncObj *g_lock = nullptr;

bool DevMgr::m_initialized = false;

#define NO_VERIFY 1 // Use fast method to get DevObj

// TODO add missing locking and reimplement with map
// This is meant just to work for now. It hs not been optimized yet.

static DFPointerList g_driver_list;

class WaitList
{
public:
	WaitList(UpdateList &in_set, UpdateList &out_set) :
		m_in_set(in_set),
		m_out_set(out_set)
	{}
	~WaitList() {}

	UpdateList 	&m_in_set;
	UpdateList 	&m_out_set;
	SyncObj 	m_lock;
};

static DFPointerList g_wait_list;

int DevMgr::initialize(void)
{
	g_lock = new SyncObj();

	if (g_lock == nullptr) {
		return -3;
	}

	m_initialized = true;
	return 0;
}

void DevMgr::finalize(void)
{
	g_lock->lock();
	m_initialized = false;

	g_driver_list.clear();

	g_wait_list.clear();

	g_lock->unlock();
	delete g_lock;
	g_lock = nullptr;
}

#if DRIVER_MAX_INSTANCES > 9
#error must redo algorithm below
#endif

int DevMgr::registerDriver(DevObj *obj)
{
	DF_LOG_DEBUG("DevMgr::registerDriver %s", obj->m_name);

	if (obj->m_dev_path == nullptr) {
		return -2;
	}

	bool registered = false;
	g_lock->lock();
	int ret = 0;

	if (obj->m_dev_class_path != nullptr) {
		for (unsigned int i = 0; i < DRIVER_MAX_INSTANCES; i++) {
			bool found = false;
			char tmp_path[strlen(obj->m_dev_class_path) + 3];
			snprintf(tmp_path, sizeof(tmp_path), "%s%d", obj->m_dev_class_path, i);
			DFPointerList::Index idx = nullptr;
			idx = g_driver_list.next(idx);

			while (idx != nullptr) {
				DevObj *list_obj = reinterpret_cast<DevObj *>(g_driver_list.get(idx));

				if ((list_obj->m_dev_instance_path) && (strcmp(tmp_path, list_obj->m_dev_instance_path) == 0)) {
					found = true;
					break;
				}

				idx = g_driver_list.next(idx);
			}

			if (!found)  {
				if (obj->m_dev_instance_path) {
					free(obj->m_dev_instance_path);
					obj->m_dev_instance_path = nullptr;
				}

				obj->m_dev_instance_path = strdup(tmp_path);
				g_driver_list.pushBack(obj);
				DF_LOG_INFO("Added driver %p %s", obj, obj->m_dev_instance_path);
				registered = true;
				break;
			}
		}

		if (!registered) {
			// Error - no available dev class instance
			ret = -3;
		}

	} else {
		// Some drivers do not specify a base class, or hardcode a specific instance
		g_driver_list.pushBack(obj);
		DF_LOG_INFO("Added driver %p %s", obj, obj->m_dev_path);
	}

	g_lock->unlock();
	return ret;
}

void DevMgr::unregisterDriver(DevObj *obj)
{
	DF_LOG_DEBUG("DevMgr::unregisterDriver %s", obj->m_name);

	if (!g_lock) {
		return;
	}

	g_lock->lock();
	DFPointerList::Index idx = nullptr;
	idx = g_driver_list.next(idx);

	while (idx != nullptr) {
		DevObj *list_obj = reinterpret_cast<DevObj *>(g_driver_list.get(idx));

		if (list_obj == obj) {
			g_driver_list.erase(idx);
			break;
		}

		idx = g_driver_list.next(idx);
	}

	g_lock->unlock();
}

DevObj *DevMgr::getDevObjByID(union DeviceId id)
{
	g_lock->lock();
	DFPointerList::Index idx = nullptr;
	idx = g_driver_list.next(idx);

	while (idx != nullptr) {
		DevObj *list_obj = reinterpret_cast<DevObj *>(g_driver_list.get(idx));

		if (list_obj->getId().dev_id == id.dev_id) {
			break;
		}

		idx = g_driver_list.next(idx);
	}

	g_lock->unlock();
	return (idx != nullptr) ? reinterpret_cast<DevObj *>(g_driver_list.get(idx)) : nullptr;
}

DevObj *DevMgr::_getDevObjByHandle(DevHandle &h)
{
	g_lock->lock();
	DFPointerList::Index idx = nullptr;
	idx = g_driver_list.next(idx);

	while (idx != nullptr) {
		DevObj *list_obj = reinterpret_cast<DevObj *>(g_driver_list.get(idx));

		if (h.m_handle == list_obj) {
			break;
		}

		idx = g_driver_list.next(idx);
	}

	g_lock->unlock();
	return (idx == nullptr) ? nullptr : reinterpret_cast<DevObj *>(g_driver_list.get(idx));
}

void DevMgr::getHandle(const char *dev_path, DevHandle &h)
{
	if (dev_path == nullptr) {
		h.m_errno = EINVAL;
		return;
	}

	h.m_errno = EBADF;

	g_lock->lock();
	DFPointerList::Index idx = nullptr;
	idx = g_driver_list.next(idx);

	while (idx != nullptr) {
		DevObj *list_obj = reinterpret_cast<DevObj *>(g_driver_list.get(idx));

		// The dev path may be a class instance path or a dev name
		if ((strcmp(dev_path, list_obj->m_dev_path) == 0) ||
		    (list_obj->m_dev_instance_path && (strcmp(dev_path, list_obj->m_dev_instance_path) == 0))) {

			// Device is registered
			g_lock->unlock();
			list_obj->addHandle(h);
			g_lock->lock();
			h.m_handle = list_obj;
			h.m_errno = 0;
			break;
		}

		idx = g_driver_list.next(idx);
	}

	g_lock->unlock();
}

void DevMgr::releaseHandle(DevHandle &h)
{
	DevObj *driver = DevMgr::getDevObjByHandle<DevObj>(h);

	if (driver) {
		driver->removeHandle(h);
	}

	h.m_handle = nullptr;
	h.m_errno = 0;
}

void DevMgr::setDevHandleError(DevHandle &h, int error)
{
	h.m_errno = error;
}

int DevMgr::getNextDeviceName(unsigned int &index, const char **instancename)
{
	unsigned int i = 0;
	int ret = -1;
	g_lock->lock();

	// First go through actual dev names
	DFPointerList::Index idx = nullptr;
	idx = g_driver_list.next(idx);

	while (idx != nullptr) {
		if (i == index) {
			DevObj *list_obj = reinterpret_cast<DevObj *>(g_driver_list.get(idx));
			*instancename = list_obj->m_dev_instance_path;
			index += 1;
			ret = 0;
			break;
		}

		idx = g_driver_list.next(idx);
		++i;
	}

	g_lock->unlock();
	return ret;
}

int DevMgr::waitForUpdate(UpdateList &in_set, UpdateList &out_set, unsigned int timeout_ms)
{
	WaitList wl(in_set, out_set);

	g_lock->lock();	  // HERE M7
	wl.m_lock.lock(); // HERE M11
	g_wait_list.pushFront(&wl);
	g_lock->unlock();

	int ret = wl.m_lock.waitOnSignal(timeout_ms);
	wl.m_lock.unlock();

	// Remove the List item
	g_lock->lock();
	DFPointerList::Index idx = nullptr;
	idx = g_wait_list.next(idx);

	while (idx != nullptr) {
		WaitList *lwl = reinterpret_cast<WaitList *>(g_wait_list.get(idx));

		if (lwl == &wl) {
			idx = g_wait_list.erase(idx);
			break;
		}

		idx = g_wait_list.next(idx);
	}

	g_lock->unlock();

	return ret;
}

void  DevMgr::updateNotify(DevObj &obj)
{
	g_lock->lock(); // HERE M7
	DFPointerList::Index idx = nullptr;
	idx = g_wait_list.next(idx);

	while (idx != nullptr) {
		WaitList *lwl = reinterpret_cast<WaitList *>(g_wait_list.get(idx));

		DFPointerList::Index it = nullptr;
		it = lwl->m_in_set.next(it);

		while (it != nullptr) {
			DevHandle *h = reinterpret_cast<DevHandle *>(lwl->m_in_set.get(it));

			// If the obj is equal the obj of DevHandle
			if (h->m_handle == &obj) {

				// Add obj to the out set
				lwl->m_out_set.pushBack(h);
			}

			lwl->m_in_set.next(it);
		}

		if (!lwl->m_out_set.empty()) {
			lwl->m_lock.lock(); // HERE M11
			lwl->m_lock.signal();
			lwl->m_lock.unlock();
		}

		idx = g_wait_list.next(idx);
	}

	g_lock->unlock();
}

//------------------------------------------------------------------------
// DevHandle
//------------------------------------------------------------------------

DevHandle::~DevHandle()
{
	if (isValid()) {
		DevMgr::releaseHandle(*this);
	}
}

#ifndef __DF_NUTTX

int DevHandle::ioctl(unsigned long cmd, unsigned long arg)
{
	if (m_handle) {
		return reinterpret_cast<DevObj *>(m_handle)->devIOCTL(cmd, arg);
	}

	return -1;
}

ssize_t DevHandle::read(void *buf, size_t len)
{
	if (m_handle) {
		return reinterpret_cast<DevObj *>(m_handle)->devRead(buf, len);
	}

	return -1;
}

ssize_t DevHandle::write(const void *buf, size_t len)
{
	if (m_handle) {
		return reinterpret_cast<DevObj *>(m_handle)->devWrite(buf, len);
	}

	return -1;
}

#else

int DevHandle::ioctl(unsigned long cmd, unsigned long arg)
{
	return ::ioctl(m_fd, cmd, (void *)arg);
}

ssize_t DevHandle::read(void *buf, size_t len)
{
	return ::read(m_fd, buf, len);
}

ssize_t DevHandle::write(const void *buf, size_t len)
{
	return ::write(m_fd, buf, len);
}

#endif
