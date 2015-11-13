#include <string.h>
#include "SyncObj.hpp"
#include "VirtDevObj.hpp"

#define TEST_DRIVER_PATH "/dev/foo"
#define TEST_DRIVER_CLASS_PATH "/dev/test"

#define TEST_IOCTL_CMD 		1
#define TEST_IOCTL_RESULT 	10

using namespace DriverFramework;

struct TestMessage {
	int val;
};

class TestDriver : public VirtDevObj
{
public:
	TestDriver() :
		VirtDevObj("TestDriver", TEST_DRIVER_PATH, TEST_DRIVER_CLASS_PATH, 50000),
		m_count(sizeof(m_message)/sizeof(m_message[0]))
	{}
	virtual ~TestDriver() {}

	// New way to read Device or Device subclass specific APIs
	static int readMessages(DevHandle &h, TestMessage *m, unsigned int count)
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

	// Alternate (old) way to read
	virtual ssize_t devRead(void *buf, size_t len)
	{
		if (len > sizeof(m_message))
		{
			len = sizeof(m_message);
		}
		m_lock.lock();
		memcpy(buf, m_message, len);
		m_lock.unlock();
		return len;
	}

	virtual int devIOCTL(unsigned long cmd, unsigned long arg)
	{
		if (cmd == TEST_IOCTL_CMD)
		{
			*reinterpret_cast<int *>(arg) = TEST_IOCTL_RESULT;
			return 0;
		}
		return -1;
	}

protected:
	virtual void _measure()
	{
		static int i = 0;
		m_lock.lock();
		m_message[i % m_count].val = i;
		i++;
		updateNotify();
		m_lock.unlock();
	}

	SyncObj		m_lock;
	TestMessage 	m_message[3];
	unsigned int 	m_count;
};

