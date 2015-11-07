#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include "DriverFramework.hpp"
#include "DevMgr.hpp"
#include "testdriver.hpp"

using namespace DriverFramework;

static void printMessages(TestMessage *message, unsigned int count)
{
	DF_LOG_INFO("Read %d messages", count);
	for (unsigned i = 0; i < count; i++) {
		DF_LOG_INFO("message %d: %d", i, message[i].val);
	}
}

void showResult(int result, bool expected_pass)
{
	DF_LOG_INFO("test %s (%d)", (((result != 0) && !expected_pass) || ((result == 0) && expected_pass)) ? "PASSED" : "FAILED", result);
}

static void test_read(DevHandle &h1, DevHandle &h2, unsigned int timeout_ms, bool expected_pass)
{
	TestMessage message[5];

	UpdateList in_set, out_set;
	in_set.push_back(&h1);
	in_set.push_back(&h2);

	int result = DevMgr::waitForUpdate(in_set, out_set, timeout_ms);
	if (result == 0) {
		UpdateList::iterator it = out_set.begin();
		for (; it != out_set.end(); ++it) {
			int len = (*it)->read(&message, sizeof(message));
			printMessages(message, len/sizeof(message[0]));
		}
	}
	else {
		DF_LOG_INFO("timed out: %s (ret: %d)", strerror(result), result);
	}
	showResult(result, expected_pass);
}

int main()
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

	sleep(1);

	const std::string devname = std::string(TEST_DRIVER_CLASS_PATH) + std::to_string(0);
	DevHandle h;
	DevMgr::getHandle(devname.c_str(), h);

	if (!h.isValid()) {
		DF_LOG_INFO("Failed to open %s (%d)", devname.c_str(), h.getError());
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

		sleep(1);

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
		int result;
		if (h.ioctl(TEST_IOCTL_CMD, &result) < 0 || result != TEST_IOCTL_RESULT) {
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

	}
	DF_LOG_INFO("tests done");
	test.stop();

	DF_LOG_INFO("shutdown");
	Framework::shutdown();

	return 0;
}
