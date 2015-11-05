#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include "DriverFramework.hpp"
#include "DevMgr.hpp"
#include "testdriver.hpp"

using namespace DriverFramework;

static void printMessages(TestMessage *message, unsigned int count)
{
	printf("Read %d messages\n", count);
	for (unsigned i = 0; i < count; i++) {
		printf("message %d: %d\n", i, message[i].val);
	}
}

void showResult(int result, bool expected_pass)
{
	printf("test %s (%d)\n", (((result != 0) && !expected_pass) || ((result == 0) && expected_pass)) ? "PASSED" : "FAILED", result);
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
		printf("timed out %d\n", result);
	}
	showResult(result, expected_pass);
}

int main()
{
	int ret = Framework::initialize();
	if (ret < 0) {
		printf("Framework::initialize() failed\n");
		return ret;
	}

	TestDriver test;

	// Register the driver
	ret = test.init();
	if (ret < 0) {
		printf("init() failed (%d))\n", ret);
		return ret;
	}

	// Start the driver
	ret = test.start();
	if (ret < 0) {
		printf("start() failed (%d)\n", ret);
		return ret;
	}

	sleep(1);

	const std::string devname = std::string(TEST_DRIVER_CLASS_PATH) + std::to_string(0);
	DevHandle h;
	DevMgr::getHandle(devname.c_str(), h);

	if (!h.isValid()) {
		printf("Failed to open %s (%d)\n", devname.c_str(), h.getError());
	}
	else {
		TestMessage message[5];

		printf("TEST 1: readMessages\n");
		int count = TestDriver::readMessages(h, message, 5);
		int ret = 0;
		if (count < 0) {
			printf("Failed to readMessages from TestDriver (%d)\n", h.getError());
			ret = 1;
		}
		else {
			printMessages(message, count);
		}
		showResult(ret, true);

		sleep(1);

		printf("TEST 2: read\n");
		ret = 0;
		int len = h.read(&message, sizeof(message));
		printf("Read %d bytes (message = %zu bytes)\n", len, sizeof(message[0]));
		if (len <= 0) { 
			printf("Failed to read from adcsim (%d)\n", h.getError());
			ret = 1;
		}
		else {
			printMessages(message, len/sizeof(message[0]));
		}
		showResult(ret, true);

		printf("TEST 3: ioctl\n");
		ret = 0;
		int result;
		if (h.ioctl(TEST_IOCTL_CMD, &result) < 0 || result != TEST_IOCTL_RESULT) {
			printf("ioctl failed (%d)\n", h.getError());
			ret = 1;
		}
		else {
			printf("ioctl TEST_IOCTL_RESULT received\n");
		}
		showResult(ret, true);

		printf("TEST 4: Polling blocking read\n");
		ret = 0;
		DevHandle h2;
		DevMgr::getHandle(TEST_DRIVER_PATH, h2);
		if (!h2.isValid()) {
			printf("Failed to open %s (%d)\n", TEST_DRIVER_PATH, h2.getError());
		}
		test_read(h, h2, 0, true);

		printf("TEST 5: Polling timeout\n");
		printf("Stopping driver and waiting for timeout\n");
		test.stop();
		// wait for scheduled work to expire
		printf("reading\n");
		test_read(h, h2, 1000, false);

		printf("TEST 6: Read after setSampleInterval\n");
		test.setSampleInterval(100000);
		test.start();
		test_read(h, h2, 1000, true);


	}
	test.stop();

	Framework::shutdown();

	return 0;
}
