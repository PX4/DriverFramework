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

	virtual int start(void)
	{
		m_work_handle = WorkMgr::create(measure, this, 10000);
		WorkMgr::schedule(m_work_handle);
		return 0;
	}

	virtual int stop(void) {
		WorkMgr::destroy(m_work_handle);
		m_work_handle=0;
		return 0;
	}

private:
	static void measure(void *arg, const WorkHandle wh)
	{
		WorkMgr::schedule(wh);
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
	asim.start();

	DriverFramework::waitForShutdown();

	return 0;
}
