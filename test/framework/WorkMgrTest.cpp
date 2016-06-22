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
#include <unistd.h>
#include "WorkMgrTest.hpp"

using namespace DriverFramework;

static SyncObj *cb_counter;

static uint64_t cb_times[3];

void WorkMgrTest::_doTests()
{
	TestDriver test;

	cb_counter = new SyncObj;
	reportResult("Verify Schedule", verifySchedule());

	test.stop();
	delete cb_counter;
}

static void callback(void *arg)
{
	cb_counter->lock();
	int *x = reinterpret_cast<int *>(arg);

	if (*x < 3) {
		cb_times[*x] = offsetTime();
		*x += 1;
	}

	cb_counter->unlock();
}

static bool verifyDelay(WorkHandle &h, uint32_t delay_usec, int *arg)
{
	uint64_t starttime = offsetTime();

	cb_counter->lock();
	*arg = 0;
	int ret = WorkMgr::schedule(h);

	if (ret != 0) {
		DF_LOG_ERR("Schedule failed (%d)", ret);
		return false;
	}

	cb_counter->unlock();

#ifdef __APPLE__
	// We need to be generous on Mac.
	const unsigned tolerance_us = 1500;
#else
	const unsigned tolerance_us = 500;
#endif

	usleep((delay_usec + tolerance_us) * 3);
	cb_counter->lock();
	WorkMgr::releaseWorkHandle(h);

	if (*arg < 3) {
		DF_LOG_ERR("Failed to get 3 callbacks (%d)", *arg);
		cb_counter->unlock();
		return false;
	}

	cb_counter->unlock();

	for (int i = 0; i < 3; i++) {
		uint64_t elapsedtime = cb_times[i] - starttime;

		// Shouldn't take too much longer.
		uint64_t time_to_achieve = (delay_usec + tolerance_us) * (i + 1);

		DF_LOG_INFO("Delay: %uusec Expected: %" PRIu64 " Actual: %" PRIu64 " Delta: %ldusec",
			    delay_usec * (i + 1), starttime + delay_usec * (i + 1), cb_times[i],
			    (long)(cb_times[i] - starttime - (delay_usec * (i + 1))));

		if (elapsedtime > time_to_achieve) {
			DF_LOG_ERR("Failed %u usec timeout * %d (%" PRIu64 " > %" PRIu64 ")",
				   delay_usec, (i + 1), elapsedtime, time_to_achieve);
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
		WorkMgr::getWorkHandle(callback, &arg, delay_usec, h);

		if (!h.isValid()) {
			DF_LOG_ERR("getWorkHandle failed for delay of %u", delay_usec);
			return false;
		}

		if (!verifyDelay(h, delay_usec, &arg)) {
			return false;
		}
	}

	WorkMgr::releaseWorkHandle(h);

	return true;
}

