#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include "DriverFramework.hpp"
#include "DevMgr.hpp"
#include "testdriver.hpp"

using namespace DriverFramework;

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
			printf("Failed to read from adcsim (%d)\n", h.getError());
		}
		else {
			printf("Read %d messages\n", count);
			for (int i=0; i<count; i++) {
				printf("message %d: %d\n", i, message[i].val);
			}
		}
	}
	test.stop();

	Framework::shutdown();

	return 0;
}
