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
#include "testdriver.hpp"

using namespace DriverFramework;

extern void ListTests(void);

static void printMessages(TestMessage *message, unsigned int count)
{
	DF_LOG_INFO("Read %d messages", count);
	for (unsigned i = 0; i < count; i++) {
		DF_LOG_INFO("message %d: %d", i, message[i].val);
	}
}

static void showResult(int result, bool expected_pass)
{
	DF_LOG_INFO("test %s (%d)", (((result != 0) && !expected_pass) || ((result == 0) && expected_pass)) ? "PASSED" : "FAILED", result);
}

static void test_read(DevHandle &h1, DevHandle &h2, unsigned int timeout_ms, bool expected_pass)
{
	TestMessage message[5];

	UpdateList in_set, out_set;
	in_set.pushBack(&h1);
	in_set.pushBack(&h2);

	int result = DevMgr::waitForUpdate(in_set, out_set, timeout_ms);
	if (result == 0) {
		DFPointerList::Index idx = nullptr;
		idx = out_set.next(idx);
		while (idx != nullptr) {
			DevHandle *h = reinterpret_cast<DevHandle *>(out_set.get(idx));
			int len = h->read(&message, sizeof(message));
			printMessages(message, len/sizeof(message[0]));
			idx = out_set.next(idx);
		}
	}
	else {
		DF_LOG_INFO("timed out: %s (ret: %d)", strerror(result), result);
	}
	showResult(result, expected_pass);
}

int do_test()
{
	int ret = Framework::initialize();
	if (ret < 0) {
		DF_LOG_INFO("Framework::initialize() failed");
		return ret;
	}

	TestDriver test;

	// Register the driver
	ret = test.init();
	if (ret < 0) {
		DF_LOG_INFO("init() failed (%d))", ret);
		return ret;
	}

	// Start the driver
	ret = test.start();
	if (ret < 0) {
		DF_LOG_INFO("start() failed (%d)", ret);
		return ret;
	}

	usleep(1000000);

	char devname[strlen(TEST_DRIVER_CLASS_PATH)+3];
	snprintf(devname, sizeof(devname), "%s%d", TEST_DRIVER_CLASS_PATH, 0);
	DevHandle h;
	DevMgr::getHandle(devname, h);

	if (!h.isValid()) {
		DF_LOG_INFO("Failed to open %s (%d)", devname, h.getError());
	}
	else {
		TestMessage message[5];

		DF_LOG_INFO("TEST 1: readMessages");
		int count = TestDriver::readMessages(h, message, 5);
		int ret = 0;
		if (count < 0) {
			DF_LOG_INFO("Failed to readMessages from TestDriver (%d)", h.getError());
			ret = 1;
		}
		else {
			printMessages(message, count);
		}
		showResult(ret, true);

		usleep(1000000);

		DF_LOG_INFO("TEST 2: read");
		ret = 0;
		int len = h.read(&message, sizeof(message));
		DF_LOG_INFO("Read %d bytes (message = %zu bytes)", len, sizeof(message[0]));
		if (len <= 0) { 
			DF_LOG_INFO("Failed to read from adcsim (%d)", h.getError());
			ret = 1;
		}
		else {
			printMessages(message, len/sizeof(message[0]));
		}
		showResult(ret, true);

		DF_LOG_INFO("TEST 3: ioctl");
		ret = 0;
		int result = h.ioctl(TEST_IOCTL_CMD, 0);

		if (result != TEST_IOCTL_RESULT) {
			DF_LOG_INFO("ioctl failed (%d)", h.getError());
			ret = 1;
		}
		else {
			DF_LOG_INFO("ioctl TEST_IOCTL_RESULT received");
		}
		showResult(ret, true);

		DF_LOG_INFO("TEST 4: Polling blocking read");
		ret = 0;
		DevHandle h2;
		DevMgr::getHandle(TEST_DRIVER_PATH, h2);
		if (!h2.isValid()) {
			DF_LOG_INFO("Failed to open %s (%d)", TEST_DRIVER_PATH, h2.getError());
		}
		test_read(h, h2, 0, true);

		DF_LOG_INFO("TEST 5: Polling timeout");
		DF_LOG_INFO("Stopping driver and waiting for timeout");
		test.stop();
		// wait for scheduled work to expire
		DF_LOG_INFO("reading");
		test_read(h, h2, 1000, false);

		DF_LOG_INFO("TEST 6: Read after setSampleInterval");
		test.setSampleInterval(100000);
		DF_LOG_INFO("start");
		test.start();
		DF_LOG_INFO("reading");
		test_read(h, h2, 1000, true);

		unsigned int index = 0;
		const char *devname;
		DF_LOG_INFO("Devices:");
		while (DevMgr::getNextDeviceName(index, &devname) == 0) {
			DF_LOG_INFO("    %s", devname);
		}


	}
	ListTests();
	DF_LOG_INFO("tests done");
	test.stop();

	DF_LOG_INFO("shutdown");
	Framework::shutdown();

	return 0;
}
