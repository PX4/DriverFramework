#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include "DriverFramework.hpp"
#include "VirtDriverObj.hpp"

using namespace DriverFramework;

class AccelSim : public VirtDriverObj
{
public:
	AccelSim() :
		VirtDriverObj("AccelSim", "/dev/accel0"),
		m_work_handle(0)
	{}
	~AccelSim() {}

	virtual int open(int flags, mode_t mode)
	{
		m_work_handle = WorkItemMgr::create(measure, this, 10000);
		WorkItemMgr::schedule(m_work_handle);
		return 0;
	}

	virtual int close(void) {
		WorkItemMgr::destroy(m_work_handle);
		return 0;
	}

	virtual int ioctl(unsigned long request, void *data)
	{
		return -1;
	}

	static void *main(void *arg)
	{
		if (m_instance != nullptr)
			return nullptr;

		m_instance = new AccelSim();
		m_instance->open(O_RDWR, 0);

		return nullptr;
	}

private:
	static void measure(void *arg, const WorkHandle wh)
	{
		WorkItemMgr::schedule(wh);
	}

	static AccelSim *m_instance;
	WorkHandle m_work_handle;
};

AccelSim *AccelSim::m_instance = nullptr;

int main()
{
	pthread_t tid;

	int ret = DriverFramework::initialize();
	if (ret < 0) {
		return ret;
	}

	ret = pthread_create(&tid, NULL, AccelSim::main, NULL);
	if (ret < 0) {
		return ret;
	}

	void *val;
	pthread_join(tid, &val);

	DriverFramework::waitForShutdown();

	return 0;
}
