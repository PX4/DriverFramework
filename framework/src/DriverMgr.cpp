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
#include "DriverObj.hpp"
#include "DriverMgr.hpp"

using namespace DriverFramework;

SyncObj *g_lock = nullptr;

bool DriverMgr::m_initialized = false;

#define NO_VERIFY 1 // Use fast method to get DriverObj

// TODO add missing locking and reimplement with map
// This is meant just to work for now. It hs not been optimized yet.

static std::list<DriverFramework::DriverObj *> *g_driver_list = nullptr;

DriverHandle::~DriverHandle()
{
	if(isValid()) {
		DriverMgr::releaseHandle(*this);
	}
}

int DriverMgr::initialize(void)
{
	g_driver_list = new std::list<DriverFramework::DriverObj *>;
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

void DriverMgr::finalize(void)
{
	if (g_driver_list == nullptr) {
		return;
	}
	g_lock->lock();
	m_initialized = false;
	std::list<DriverFramework::DriverObj *>::iterator it = g_driver_list->begin(); 
	while (it != g_driver_list->end()) {
		it = g_driver_list->erase(it);
	}
	delete g_driver_list;
	g_driver_list = nullptr;
	g_lock->unlock();
	delete g_lock;
	g_lock = nullptr;
}

int DriverMgr::registerDriver(DriverObj *obj)
{
	if (g_driver_list == nullptr) {
		return -1;
	}

	int found = false;
	g_lock->lock();
	for (unsigned int i=0; i < DRIVER_MAX_INSTANCES; i++)
	{
		std::string tmp_path = obj->m_dev_base_path + std::to_string(i);;
		std::list<DriverFramework::DriverObj *>::iterator it = g_driver_list->begin();
		while (it != g_driver_list->end()) {
			if ( tmp_path == (*it)->m_dev_instance_path) {
				found = true;
				break;
			}
		}
		if (!found)  {
			obj->m_dev_instance_path = tmp_path;
			g_driver_list->push_back(obj);
			printf("Added driver %p %s\n", obj, obj->m_dev_instance_path.c_str());
			break;
		}
	}
	g_lock->unlock();
	return 0;
}

void DriverMgr::unregisterDriver(DriverObj *obj)
{
	if (g_driver_list == nullptr) {
		return;
	}
	g_lock->lock();
	std::list<DriverFramework::DriverObj *>::iterator it = g_driver_list->begin(); 
	while (it != g_driver_list->end()) {
		if (*it == obj) {
			printf("unregisterDriver erase %p\n", *it);
			g_driver_list->erase(it);
			break;
		}
	}
	g_lock->unlock();
}

DriverObj *DriverMgr::getDriverObjByName(const std::string &name, unsigned int instance)
{
	if (g_driver_list == nullptr) {
		return nullptr;
	}
	g_lock->lock();
	std::list<DriverFramework::DriverObj *>::iterator it = g_driver_list->begin(); 
	while (it != g_driver_list->end()) {
		if ((*it)->m_name == name) {
			// see if instance matches
			const std::string tmp_path = (*it)->m_dev_base_path + std::to_string(instance);
			if (tmp_path == (*it)->m_dev_instance_path) {
				break;
			}
		}
	}
	g_lock->unlock();
	return (it != g_driver_list->end()) ? *it : nullptr;
}

DriverObj *DriverMgr::getDriverObjByID(union DeviceId id)
{
	if (g_driver_list == nullptr) {
		return nullptr;
	}
	g_lock->lock();
	std::list<DriverFramework::DriverObj *>::iterator it = g_driver_list->begin(); 
	while (it != g_driver_list->end()) {
		if ((*it)->getId().dev_id == id.dev_id) {
			g_lock->unlock();
			break;
		}
	}
	g_lock->unlock();
	return (it == g_driver_list->end()) ? nullptr : *it;
}

DriverObj *DriverMgr::_getDriverObjByHandle(DriverHandle &h)
{
	g_lock->lock();
	std::list<DriverFramework::DriverObj *>::iterator it = g_driver_list->begin();
	while (it != g_driver_list->end()) {
		if (h.m_handle == *it)
			break;
	}

	g_lock->unlock();
	return (it == g_driver_list->end()) ? nullptr : *it;
}

DriverHandle DriverMgr::getHandle(const char *dev_path)
{
	DriverHandle h;
	if (g_driver_list == nullptr) {
		h.m_errno = ESRCH;
		return h;
	}
	const std::string name(dev_path);
	h.m_errno = EBADF;

	g_lock->lock();
	std::list<DriverFramework::DriverObj *>::iterator it = g_driver_list->begin();
	while (it != g_driver_list->end()) {
		printf("m_dev_instance_path = %s\n", (*it)->m_dev_instance_path.c_str());
		if ( name == (*it)->m_dev_instance_path) {
			// Device is registered
			(*it)->incrementRefcount();
			h.m_handle = *it;
			h.m_errno = 0;
			break;
		}
		++it;
	}
	g_lock->unlock();
	return h;
}

void DriverMgr::releaseHandle(DriverHandle &h)
{
	DriverObj *driver = DriverMgr::getDriverObjByHandle<DriverObj>(h);
	if (driver) {
		driver->decrementRefcount();
		h.m_handle = nullptr;
	}
}

void DriverMgr::setDriverHandleError(DriverHandle &h, int error)
{
	h.m_errno = error;
}
