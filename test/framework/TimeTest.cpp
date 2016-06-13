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
#include "TimeTest.hpp"

bool TimeTest::verifyOffsetTime()
{

#ifdef __APPLE__
	const float error_factor = 1.5f;
#else
	const float error_factor = 1.2f;
#endif

	bool passed = true;
	uint64_t st = offsetTime();
	usleep(1000);
	uint64_t us = offsetTime();
	uint64_t delta = us - st;

	// Verify the delta was not less than sleep time and was close to sleep time
	if ((delta < 1000) || (delta > 1000 * error_factor)) {
		DF_LOG_INFO("Usleep of 1000us reported delta of %" PRIu64 "us", delta);
		passed = false;
	}

	sleep(1);

	uint64_t sl = offsetTime();
	delta = sl - us;

	// Verify the delta was not less than sleep time and was close to sleep time
	if ((delta < 1000000) || (delta > 1000000 * error_factor)) {
		DF_LOG_INFO("Sleep of 1s reported delta of %" PRIu64 "us", delta);
		passed = false;
	}

	return passed;
}

bool TimeTest::verifyAbsoluteTime()
{
	struct timespec ts, ts2;

	int rv = absoluteTime(ts);

	if (rv) {
		DF_LOG_ERR("absoluteTime failed (%d)", rv);
		return false;
	}

	sleep(2);

	rv = absoluteTime(ts2);

	if (rv) {
		DF_LOG_ERR("absoluteTime failed (%d)", rv);
		return false;
	}

	struct timespec delta = { ts2.tv_sec - ts.tv_sec, ts2.tv_nsec - ts.tv_nsec };

	int64_t seconds = delta.tv_sec + delta.tv_nsec / 1000000000;

	int64_t usecs = delta.tv_sec * 1000000 + delta.tv_nsec / 1000 - seconds * 1000000;

	// sleep should be accurate withing 50000us
	if (seconds != 2 || usecs > 50000) {
		DF_LOG_ERR("sleep(2) took %" PRId64 "sec %" PRId64 "us", seconds, usecs);
		return false;
	}

	return true;
}

bool TimeTest::verifyAbsoluteTimeInFuture()
{
	struct timespec ts, ts2;

	(void)absoluteTimeInFuture(2000000, ts);

	sleep(2);

	(void)absoluteTime(ts2);

	struct timespec delta = { ts2.tv_sec - ts.tv_sec, ts2.tv_nsec - ts.tv_nsec };

	int64_t seconds = delta.tv_sec + delta.tv_nsec / 1000000000;
	int64_t usecs = delta.tv_sec * 1000000 + delta.tv_nsec / 1000 - seconds * 1000000;

	// sleep should be accurate withing 50000us
	if (seconds != 0 || usecs > 50000) {
		DF_LOG_ERR("sleep(2) took an extra %" PRId64 "sec %" PRId64 "us", seconds, usecs);
		return false;
	}

	return true;
}

void TimeTest::_doTests()
{
	reportResult("Verify offsetTime()", verifyOffsetTime());
	reportResult("Verify verifyAbsoluteTime()", verifyAbsoluteTime());
	reportResult("Verify verifyAbsoluteTimeInFuture()", verifyAbsoluteTimeInFuture());
}
