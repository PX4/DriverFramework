#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include "DriverFramework.hpp"
#include "VirtDriverObj.hpp"

using namespace DriverFramework;

#define ACCEL_DEV_PATH "/dev/accel0"

class AccelSim : public VirtDriverObj
{
public:
	AccelSim(const char *dev_path) :
		VirtDriverObj("AccelSim", dev_path)
	{}
	~AccelSim() {}

	virtual int open(int flags)
	{
		if (flags == O_RDONLY) {
			m_work_handle = WorkItemMgr::create(measure, this, 10000);
			WorkItemMgr::schedule(m_work_handle);
			return 0;
		}
		errno = EPERM;
		return -1;
	}

	virtual int close(void) {
		WorkItemMgr::destroy(m_work_handle);
		m_work_handle=0;
		return 0;
	}

	virtual int ioctl(unsigned long request, void *data)
	{
		return -1;
	}

private:
	static void measure(void *arg, const WorkHandle wh)
	{
		WorkItemMgr::schedule(wh);
	}

	static AccelSim *m_instance;
	WorkHandle m_work_handle	= 0;
};

AccelSim *AccelSim::m_instance = nullptr;

int main()
{
	int ret = DriverFramework::initialize();
	if (ret < 0) {
		return ret;
	}

	AccelSim asim(ACCEL_DEV_PATH);

	// Start polling the sensor
	asim.open(O_RDONLY);

	DriverFramework::waitForShutdown();

	return 0;
}
