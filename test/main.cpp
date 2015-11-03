#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include "DriverFramework.hpp"
#include "DevMgr.hpp"
#include "testdriver.hpp"

using namespace DriverFramework;

void printMessages(TestMessage *message, unsigned int count)
{
	printf("Read %d messages\n", count);
	for (int i=0; i<count; i++) {
		printf("message %d: %d\n", i, message[i].val);
	}
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

	printf("sleeping\n");
	sleep(1);

	const std::string devname = std::string(TEST_DRIVER_DEV_PATH) + std::to_string(0);
	DevHandle h;
	printf("sleeping\n");
	DevMgr::getHandle(devname.c_str(), h);
	printf("sleeping\n");

	if (!h.isValid()) {
		printf("Failed to open %s (%d)\n", TEST_DRIVER_DEV_PATH, h.getError());
	}
	else {
		TestMessage message[5];

		printf("before readMessages\n");
		int count = TestDriver::readMessages(h, message, 5);
		printf("after readMessages\n");
		if (count < 0) {
			printf("Failed to readMessages from adcsim (%d)\n", h.getError());
		}
		else {
			printMessages(message, count);
		}

		printf("sleeping\n");
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
		UpdateList in_set, out_set;
		in_set.push_back(&h);
		in_set.push_back(&h2);
		result = DevMgr::waitForUpdate(in_set, out_set, 10000);
		if (result > 0) {
			UpdateList::iterator it = out_set.begin();
			for (; it != out_set.end(); ++it) {
				len = (*it)->read(&message, sizeof(message));
				printMessages(message, len/sizeof(message[0]));
			}
		}
	}
	printf("stop\n");
	test.stop();

	Framework::shutdown();

	return 0;
}
