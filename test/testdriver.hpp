#include "SyncObj.hpp"
#include "VirtDriverObj.hpp"

#define TEST_DRIVER_DEV_PATH "/dev/test0"

using namespace DriverFramework;

struct TestMessage {
	int val;
};

class TestDriver : public VirtDriverObj
{
public:
	TestDriver(const char *dev_path) :
		VirtDriverObj("TestDriver", dev_path),
		m_count(sizeof(m_message)/sizeof(m_message[0]))
	{}
	virtual ~TestDriver() {}

	virtual int start(void)
	{
		int ret = DriverMgr::registerDriver(this);
		if (ret) {
			return ret;
		}
		m_work_handle = WorkMgr::create(measure, this, 10000);
		WorkMgr::schedule(m_work_handle);
		return 0;
	}

	virtual int stop(void) {
		WorkMgr::destroy(m_work_handle);
		m_work_handle=0;
		return 0;
	}

	static int readMessages(DriverHandle h, TestMessage *m, unsigned int count)
	{
		DriverObj *obj = DriverMgr::validateHandle(h);
		if (obj !- nullptr) {
			TestDriver *me = reinterpret_cast<TestDriver *>(obj);
			if (count > me->m_count)
			{
				count = me->m_count;
			}
			me->m_lock.lock();
			for (int i = 0; i < count; i++) {
				m[i] = me->m_message[i];
			}
			me->m_lock.unlock();
			DriverMgr::setDriverHandleError(h, 0);
			return count;
		}
		else {
			return -1;
		}
	}

private:
	static void measure(void *arg, const WorkHandle wh)
	{
		WorkMgr::schedule(wh);
		reinterpret_cast<TestDriver *>(arg)->_measure();
		
	}

	void _measure()
	{
		static int i = 0;
		m_lock.lock();
		m_message[i % m_count].val = i;
		i++;
		m_lock.unlock();
	}

	WorkHandle 	m_work_handle	= 0;
	SyncObj		m_lock;

	TestMessage m_message[3];
	unsigned int m_count;
};

