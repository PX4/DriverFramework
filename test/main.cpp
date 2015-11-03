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
	printf("test %s (%d)\n", (((result != 0) && !expected_pass) || ((result == 0) && expected_pass)) ? "PASSED" : "FAILED", result);
}

int main()
{
	int ret = Framework::initialize();
	if (ret < 0) {
		return ret;
	}

	TestDriver test;

	// Start the driver
	test.start();

	sleep(1);

	const std::string devname = std::string(TEST_DRIVER_DEV_PATH) + std::to_string(0);
	DevHandle h;
	DevMgr::getHandle(devname.c_str(), h);

	if (!h.isValid()) {
		printf("Failed to open %s (%d)\n", TEST_DRIVER_DEV_PATH, h.getError());
	}
	else {
		TestMessage message[5];

		int count = TestDriver::readMessages(h, message, 5);
		if (count < 0) {
			printf("Failed to readMessages from adcsim (%d)\n", h.getError());
		}
		else {
			printMessages(message, count);
		}

		sleep(1);

		int len = h.read(&message, sizeof(message));
		printf("Read %d bytes (message = %zu bytes)\n", len, sizeof(message[0]));
		if (len <= 0) { 
			printf("Failed to read from adcsim (%d)\n", h.getError());
		}
		else {
			printMessages(message, len/sizeof(message[0]));
		}

		int result;
		if (h.ioctl(TEST_IOCTL_CMD, &result) < 0 || result != TEST_IOCTL_RESULT) {
			printf("ioctl failed (%d)\n", h.getError());
		}
		else {
			printf("ioctl TEST_IOCTL_RESULT received\n");
		}

		// Test polling
		DevHandle h2;
		DevMgr::getHandle(devname.c_str(), h2);
		test_read(h, h2, 0, true);

		// Test poll timeout
		printf("Stopped samples\n");
		test.stop();
		sleep(1);
		test_read(h, h2, 1000, false);

		// test reset sample rate
		test.setSampleInterval(100000);
		test.start();
		test_read(h, h2, 1000, true);


	}
	test.stop();

	Framework::shutdown();

	return 0;
}
