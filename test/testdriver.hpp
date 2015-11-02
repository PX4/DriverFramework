#include "SyncObj.hpp"
#include "VirtDevObj.hpp"

#define TEST_DRIVER_DEV_PATH "/dev/test"

using namespace DriverFramework;

struct TestMessage {
	int val;
};

class TestDriver : public VirtDevObj
{
public:
	TestDriver() :
		VirtDevObj("TestDriver", TEST_DRIVER_DEV_PATH, 100),
		m_count(sizeof(m_message)/sizeof(m_message[0]))
	{}
	virtual ~TestDriver() {}

	static int readMessages(DevHandle h, TestMessage *m, unsigned int count)
	{
		TestDriver *me = DevMgr::getDevObjByHandle<TestDriver>(h);
		if (me != nullptr) {
			if (count > me->m_count)
			{
				count = me->m_count;
			}
			me->m_lock.lock();
			for (unsigned int i = 0; i < count; i++) {
				m[i] = me->m_message[i];
			}
			me->m_lock.unlock();
			DevMgr::setDevHandleError(h, 0);
			return count;
		}
		else {
			return -1;
		}
	}

protected:
	virtual void _measure()
	{
		static int i = 0;
		m_lock.lock();
		m_message[i % m_count].val = i;
		i++;
		m_lock.unlock();
	}

	SyncObj		m_lock;
	TestMessage 	m_message[3];
	unsigned int 	m_count;
};

