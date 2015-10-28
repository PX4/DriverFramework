#include<iostream>
#include<unistd.h>
#include"DriverFramework.hpp"
#include"DriverObj.hpp"

using namespace DriverFramework;

class AccelSim : public VirtDriverObj
{
public:
	AccelSim() :
		VirtDriverObj("AccelSim", "/dev/accel0"),
		m_work_handle(0)
	{}
	~AccelSim() {}

	int initialize(void)
	{
		m_work_handle = WorkItemMgr::create(measure, this, 1000000);
		WorkItemMgr::schedule(m_work_handle);
		return 0;
	}

	int ioctl(int datatype, void *data)
	{
		return 0;
	}

	static void *main(void *arg)
	{
		if (m_instance != nullptr)
			return nullptr;

		m_instance = new AccelSim();
		m_instance->initialize();

		return nullptr;
	}

private:
	static void measure(void *arg, const WorkHandle wh)
	{
		std::cout << "Measure\n";
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
