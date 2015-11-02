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

	sleep(1);

	const std::string devname = std::string(TEST_DRIVER_DEV_PATH) + std::to_string(0);
	DevHandle h = DevMgr::getHandle(devname.c_str());
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
		
	}
	printf("stop\n");
	test.stop();

	Framework::shutdown();

	return 0;
}
