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
#include <errno.h>
#include "SyncObjTest.hpp"
#include "SyncObj.hpp"

void SyncObjTest::_doTests()
{
	SyncObj so;

	bool passed = true;
	so.lock();
	so.unlock();
	reportResult("Simple lock/unlock", passed);

	uint64_t now = offsetTime();
	unsigned long wait_in_us = 5000;
	so.lock();
	int rv = so.waitOnSignal(wait_in_us);
	so.unlock();
	uint64_t after = offsetTime();
	uint64_t delta_usec = after - now;

	if (rv != ETIMEDOUT) {
		DF_LOG_ERR("waitSignal() did not return ETIMEDOUT");
		passed = false;
	}

#ifdef __APPLE__
	const float error_factor = 2.0f;
#else
	const float error_factor = 1.2f;
#endif

	bool dtime_ok = (delta_usec > wait_in_us) && (delta_usec < wait_in_us * error_factor);

	if (!dtime_ok) {
		DF_LOG_ERR("waitSignal() timeout of %luus was %" PRIu64 "us", wait_in_us, delta_usec);
		passed = false;
	}

	reportResult("waitSignal() timeout", passed && dtime_ok);
}

