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
#pragma once

#include <string.h>
#include "SyncObj.hpp"
#include "VirtDevObj.hpp"

#define TEST_DRIVER_PATH "/dev/foo"
#define TEST_DRIVER_CLASS_PATH "/dev/test"

#define TEST_IOCTL_CMD 		1
#define TEST_IOCTL_RESULT 	10

using namespace DriverFramework;

struct TestMessage {
	int val;
};

class TestDriver : public VirtDevObj
{
public:
	TestDriver() :
		VirtDevObj("TestDriver", TEST_DRIVER_PATH, TEST_DRIVER_CLASS_PATH, 50000),
		m_count(sizeof(m_message) / sizeof(m_message[0]))
	{}
	virtual ~TestDriver() {}

	// New way to read Device or Device subclass specific APIs
	static int readMessages(DevHandle &h, TestMessage *m, unsigned int count)
	{
		TestDriver *me = DevMgr::getDevObjByHandle<TestDriver>(h);

		if (me != nullptr) {
			if (count > me->m_count) {
				count = me->m_count;
			}

			me->m_lock.lock();

			for (unsigned int i = 0; i < count; i++) {
				m[i] = me->m_message[i];
			}

			me->m_lock.unlock();
			DevMgr::setDevHandleError(h, 0);
			return count;

		} else {
			return -1;
		}
	}

	// Alternate (old) way to read
	virtual ssize_t devRead(void *buf, size_t len)
	{
		if (len > sizeof(m_message)) {
			len = sizeof(m_message);
		}

		m_lock.lock();
		memcpy(buf, m_message, len);
		m_lock.unlock();
		return len;
	}

	virtual int devIOCTL(unsigned long cmd, unsigned long arg)
	{
		if (cmd == TEST_IOCTL_CMD) {
			return TEST_IOCTL_RESULT;
		}

		return -1;
	}

protected:
	virtual void _measure()
	{
		static int i = 0;
		m_lock.lock();
		m_message[i % m_count].val = i;
		i++;
		updateNotify();
		m_lock.unlock();
	}

	SyncObj		m_lock;
	TestMessage 	m_message[3];
	unsigned int 	m_count;
};

