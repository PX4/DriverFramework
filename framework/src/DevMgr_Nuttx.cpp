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
#include <errno.h>

using namespace DriverFramework;

static SyncObj *g_lock = nullptr;

bool DevMgr::m_initialized = false;

#define NO_VERIFY 1 // Use fast method to get DevObj

// TODO add missing locking and reimplement with map
// This is meant just to work for now. It hs not been optimized yet.

static std::list<DriverFramework::DevObj *> *g_driver_list = nullptr;

int DevMgr::initialize(void)
{
	return 0;
}

void DevMgr::finalize(void)
{
}

void DevMgr::getHandle(const char *dev_path, DevHandle &h)
{
	if (dev_path == nullptr) {
		h.m_errno = EINVAL;
		return;
	}

	const std::string name(dev_path);
	h.m_errno = EBADF;

	// release the handle if it is valid
	DevMgr::releaseHandle(h);

	g_lock->lock();
	std::list<DriverFramework::DevObj *>::iterator it = g_driver_list->begin();
	while (it != g_driver_list->end()) {

		// The dev path may be a class instance path or a dev name
		if ((name == (*it)->m_dev_instance_path) || (name == (*it)->m_dev_path)) {
			// Device is registered
			// Try and open it
			h.m_fd = ::open(dev_path, O_RDONLY);

			if (h.m_fd >=0) {
				g_lock->unlock();
				(*it)->addHandle(h);
				g_lock->lock();
				h.m_handle = *it;
				h.m_errno = 0;
			} else {
				h.m_errno = errno;
			}
			break;
		}
		++it;
	}
	g_lock->unlock();
}

void DevMgr::releaseHandle(DevHandle &h)
{
	::close(h.m_fd);
	h.m_fd = -1;
	h.m_handle = nullptr;
	h.m_errno = 0;
}

void DevMgr::setDevHandleError(DevHandle &h, int error)
{
	h.m_errno = error;
}

int DevMgr::getNextDevicePath(unsigned int &index, std::string &dev_path, std::string &instance_path)
{
	int idx = 0;

	/* list directory */
	DIR *d = ::opendir("/dev");

	if (d) {
		struct dirent	*direntry;
		string devname("/dev/");

		while ((direntry = readdir(d)) != NULL) {
			if (idx == index) {
				dev_path = devname.append(direntry->d_name);
				++index;
				return 0;
			}
		}
	}

	return -1;
}

int DevMgr::waitForUpdate(UpdateList &in_set, UpdateList &out_set, unsigned int timeout_ms)
{
	// poll on a set of file descriptors
	struct pollfd fds[in_set.size()];

	size_t i = 0;
	UpdateList::iterator in_it = (*it)->in_set.begin();
	for (; in_it != (*it)->in_set.end(); ++in_it) {
		fds[i].fd = in_it->m_fd;
		fds[i].events = POLLIN;
		++i;
	}

	int ret = poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), timeout_ms);

	if (ret > 0) {
		// build the out_set
		size_t i = 0;
		UpdateList::iterator in_it = (*it)->in_set.begin();
		for (; in_it != (*it)->in_set.end(); ++in_it) {
			if (fds[i].revents & POLLIN) {
				out_set.push_back((*in_it))
			}
		}
	}

	return ret;
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

