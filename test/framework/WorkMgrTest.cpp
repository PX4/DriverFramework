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
#include "WorkMgrTest.hpp"

using namespace DriverFramework;

void WorkMgrTest::_doTests()
{
	TestDriver test;

	reportResult("Verify Schedule", verifySchedule());

	test.stop();
}

static void callback(void *arg)
{
	int *x = reinterpret_cast<int *>(arg);
	*x += 1;
}

static bool verifyDelay(WorkHandle &h, uint32_t delay_usec, int *arg)
{
	*arg = 0;
	uint64_t starttime = offsetTime();

	if (WorkMgr::schedule(h) != 0) {
		return false;
	}

	uint64_t now;
	uint64_t elapsedtime;

	while (*arg < 3) {
		now = offsetTime(); // In usec
		elapsedtime = now - starttime;

		// Shouldn't take more than extra 50us
		uint64_t time_to_achieve = delay_usec * 3 + 50;

		if (elapsedtime > time_to_achieve) {
			DF_LOG_ERR("Failed %u usec timeout * 3 (%" PRIu64 " > %" PRIu64 ")",
				   delay_usec, elapsedtime, time_to_achieve);
			return false;
		}
	}

	DF_LOG_INFO("Verified %uusec timeout", delay_usec);
	return true;
}

bool WorkMgrTest::verifySchedule()
{
	int arg = 0;
	WorkHandle h;
	uint32_t delay_usec = 0;
	WorkMgr::getWorkHandle(callback, &arg, delay_usec, h);

	if (!h.isValid()) {
		DF_LOG_ERR("getWorkHandle failed for delay of 0");
		return false;
	}

	if (!verifyDelay(h, delay_usec, &arg)) {
		return false;
	}

	delay_usec = 1;
	WorkMgr::getWorkHandle(callback, &arg, delay_usec, h);

	if (!h.isValid()) {
		DF_LOG_ERR("getWorkHandle failed for delay of 1");
		return false;
	}

	if (!verifyDelay(h, delay_usec, &arg)) {
		return false;
	}

	for (uint32_t i = 10; i < 300010; i += 10000) {
		delay_usec = i;
		int ret = WorkMgr::releaseWorkHandle(h);

		if (ret != 0) {
			DF_LOG_ERR("releaseWorkHandle failed with ret %d", ret);
		}

		WorkMgr::getWorkHandle(callback, &arg, delay_usec, h);

		if (!h.isValid()) {
			DF_LOG_ERR("getWorkHandle failed for delay of %u", delay_usec);
			return false;
		}

		if (!verifyDelay(h, delay_usec, &arg)) {
			return false;
		}
	}

	int rv = WorkMgr::releaseWorkHandle(h);

	if (rv != 0) {
		DF_LOG_ERR("Failed: releaseWorkHandle returned %d", rv);
		return false;
	}

	return true;
}

