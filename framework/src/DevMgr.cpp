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
#include <string>
#include <list>
#include "DriverFramework.hpp"
#include "SyncObj.hpp"
#include "DevObj.hpp"
#include "DevMgr.hpp"

#include <stdlib.h>
#include <execinfo.h>

using namespace DriverFramework;

SyncObj *g_lock = nullptr;

bool DevMgr::m_initialized = false;

#define NO_VERIFY 1 // Use fast method to get DevObj

// TODO add missing locking and reimplement with map
// This is meant just to work for now. It hs not been optimized yet.

static std::list<DriverFramework::DevObj *> *g_driver_list = nullptr;

class WaitList {
public:
	WaitList(UpdateList &in_set, UpdateList &out_set) :
		m_in_set(in_set),
		m_out_set(out_set)
	{}
	~WaitList() {}
		
	UpdateList &	m_in_set;
	UpdateList &	m_out_set;
	SyncObj 	m_lock;
};

static std::list<WaitList *> *g_wait_list = 0;

int DevMgr::initialize(void)
{
	g_wait_list = new std::list<WaitList *>;
	g_driver_list = new std::list<DriverFramework::DevObj *>;
	if (g_driver_list == nullptr) {
		return -1;
	}
	g_lock = new SyncObj();
	if (g_lock == nullptr) {
		delete g_driver_list;
		return -2;
	}

	m_initialized = true;
	return 0;
}

void DevMgr::finalize(void)
{
	if (g_driver_list == nullptr) {
		return;
	}
	g_lock->lock();
	m_initialized = false;
	std::list<DriverFramework::DevObj *>::iterator it = g_driver_list->begin(); 
	while (it != g_driver_list->end()) {
		it = g_driver_list->erase(it);
	}
	delete g_driver_list;
	g_driver_list = nullptr;

	delete g_wait_list;
	g_wait_list = nullptr;

	g_lock->unlock();
	delete g_lock;
	g_lock = nullptr;
}

int DevMgr::registerDriver(DevObj *obj)
{
	if (g_driver_list == nullptr) {
		return -1;
	}

	int found = false;
	g_lock->lock();
	for (unsigned int i=0; i < DRIVER_MAX_INSTANCES; i++)
	{
		std::string tmp_path = obj->m_dev_base_path + std::to_string(i);;
		std::list<DriverFramework::DevObj *>::iterator it = g_driver_list->begin();
		while (it != g_driver_list->end()) {
			if ( tmp_path == (*it)->m_dev_instance_path) {
				found = true;
				break;
			}
			++it;
		}
		if (!found)  {
			obj->m_dev_instance_path = tmp_path;
			g_driver_list->push_back(obj);
			DF_LOG_INFO("Added driver %p %s", obj, obj->m_dev_instance_path.c_str());
			break;
		}
	}
	g_lock->unlock();
	return 0;
}

void DevMgr::unregisterDriver(DevObj *obj)
{
	if (g_driver_list == nullptr) {
		return;
	}
	g_lock->lock();
	std::list<DriverFramework::DevObj *>::iterator it = g_driver_list->begin(); 
	while (it != g_driver_list->end()) {
		if (*it == obj) {
			g_driver_list->erase(it);
			break;
		}
		++it;
	}
	g_lock->unlock();
}

DevObj *DevMgr::getDevObjByName(const char *name, unsigned int instance)
{
	if (g_driver_list == nullptr) {
		return nullptr;
	}
	g_lock->lock();
	std::list<DriverFramework::DevObj *>::iterator it = g_driver_list->begin(); 
	while (it != g_driver_list->end()) {
		if ((*it)->m_name == name) {
			// see if instance matches
			const std::string tmp_path = (*it)->m_dev_base_path + std::to_string(instance);
			if (tmp_path == (*it)->m_dev_instance_path) {
				break;
			}
		}
		++it;
	}
	g_lock->unlock();
	return (it != g_driver_list->end()) ? *it : nullptr;
}

DevObj *DevMgr::getDevObjByID(union DeviceId id)
{
	if (g_driver_list == nullptr) {
		return nullptr;
	}
	g_lock->lock();
	std::list<DriverFramework::DevObj *>::iterator it = g_driver_list->begin(); 
	while (it != g_driver_list->end()) {
		if ((*it)->getId().dev_id == id.dev_id) {
			g_lock->unlock();
			break;
		}
		++it;
	}
	g_lock->unlock();
	return (it == g_driver_list->end()) ? nullptr : *it;
}

DevObj *DevMgr::_getDevObjByHandle(DevHandle &h)
{
	g_lock->lock();
	std::list<DriverFramework::DevObj *>::iterator it = g_driver_list->begin();
	while (it != g_driver_list->end()) {
		if (h.m_handle == *it) {
			break;
		}
		++it;
	}

	g_lock->unlock();
	return (it == g_driver_list->end()) ? nullptr : *it;
}

void DevMgr::getHandle(const char *dev_path, DevHandle &h)
{
	if (g_driver_list == nullptr) {
		h.m_errno = ESRCH;
		return;
	}
	const std::string name(dev_path);
	h.m_errno = EBADF;

	//g_lock->lock();
	std::list<DriverFramework::DevObj *>::iterator it = g_driver_list->begin();
	while (it != g_driver_list->end()) {
		if ( name == (*it)->m_dev_instance_path) {
			// Device is registered
			//g_lock->unlock();
			(*it)->addHandle(h);
			//g_lock->lock();
			h.m_handle = *it;
			h.m_errno = 0;
			break;
		}
		++it;
	}
	//g_lock->unlock();
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

int DevMgr::waitForUpdate(UpdateList &in_set, UpdateList &out_set, unsigned int timeout_ms)
{
	WaitList wl(in_set, out_set);

	wl.m_lock.lock();
	g_wait_list->push_front(&wl);

	int ret = wl.m_lock.waitOnSignal(timeout_ms);

	// Remove the List item
	auto it = g_wait_list->begin();
	while(it != g_wait_list->end()) {
		if (*it == &wl) {
			it = g_wait_list->erase(it);
			break;
		}
	}

	wl.m_lock.unlock();
	return ret;
}

void  DevMgr::updateNotify(DevObj &obj)
{
	std::list<WaitList *>::iterator it =  g_wait_list->begin();

	for (; it != g_wait_list->end(); ++it) {

		UpdateList::iterator in_it = (*it)->m_in_set.begin();
		for (; in_it != (*it)->m_in_set.end(); ++in_it) {

			// If the obj is equal the obj of DevHandle
			if ((*in_it)->m_handle == &obj) {

				// Add obj to the out set
				(*it)->m_out_set.push_back((*in_it));
			}
		}
		if ((*it)->m_out_set.size() > 0) {
			(*it)->m_lock.lock();
			(*it)->m_lock.signal();
			(*it)->m_lock.unlock();
		}
	}
}

//------------------------------------------------------------------------
// DevHandle
//------------------------------------------------------------------------

DevHandle::~DevHandle()
{
	if(isValid()) {
		DevMgr::releaseHandle(*this);
	}
}

int DevHandle::ioctl(unsigned long cmd, void *arg)
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
ssize_t DevHandle::write(void *buf, size_t len)
{
	if (m_handle) {
		return reinterpret_cast<DevObj *>(m_handle)->devWrite(buf, len);
	}
	return -1;
}
