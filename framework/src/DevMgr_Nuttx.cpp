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
#include <unistd.h>
#include <fcntl.h>
#include "DriverFramework.hpp"
#include "DevMgr.hpp"

#include <stdlib.h>
#include <errno.h>
#include <dirent.h>
#include <poll.h>
#include <sys/ioctl.h>

using namespace DriverFramework;

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

	h.m_errno = EBADF;

	if (h.m_fd >= 0) {
		::close(h.m_fd);
	}

	h.m_fd = ::open(dev_path, O_RDONLY);

	if (h.m_fd < 0) {
		h.m_errno = errno;
	}
}

void DevMgr::releaseHandle(DevHandle &h)
{
	::close(h.m_fd);
	h.m_fd = -1;
	h.m_errno = 0;
}

void DevMgr::setDevHandleError(DevHandle &h, int error)
{
	h.m_errno = error;
}

int DevMgr::getNextDeviceName(unsigned int &index, const char **dev_path)
{
	int idx = 0;

	/* list directory */
	DIR *d = ::opendir("/dev");

	if (d) {
		struct dirent	*direntry;

		char devname[50];

		while ((direntry = readdir(d)) != NULL) {
			if (idx == index) {
				snprintf(devname, sizeof(devname), "/dev/%s", direntry->d_name);
				*dev_path = direntry->d_name;
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
	int num = in_set.size();
	struct pollfd fds[num];

	size_t i = 0;
	DFPointerList::Index it = nullptr;
	it = in_set.next(it);

	while (it != nullptr) {
		DevHandle *h = reinterpret_cast<DevHandle *>(in_set.get(it));
		fds[i].fd = h->m_fd;
		fds[i].events = POLLIN;
		++i;
		it = in_set.next(it);
	}

	int ret = poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), timeout_ms);

	if (ret > 0) {
		// build the out_set
		size_t i = 0;
		DFPointerList::Index it = nullptr;
		it = in_set.next(it);

		while (it != nullptr) {
			DevHandle *h = reinterpret_cast<DevHandle *>(in_set.get(it));

			if (fds[i].revents & POLLIN) {
				out_set.pushBack(h);
			}

			++i;
			it = in_set.next(it);
		}
	}

	return ret;
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

int DevHandle::ioctl(unsigned long cmd, unsigned long arg)
{
	return ::ioctl(m_fd, cmd, arg);
}

ssize_t DevHandle::read(void *buf, size_t len)
{
	return ::read(m_fd, buf, len);
}

ssize_t DevHandle::write(const void *buf, size_t len)
{
	return ::write(m_fd, buf, len);
}

