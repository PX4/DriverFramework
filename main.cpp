#include<iostream>
#include"HRTWorkerThread.hpp"

class Worker
{
public:
	Worker() 
	{
		_w = std::make_shared<WorkItem>(callbackFunc, this, 1000000);
	}
	~Worker() {}

	void doWork()
	{
		HRTWorkerThread *wt = HRTWorkerThread::instance();
		wt->scheduleWorkItem(_w);
	}

private:
	std::shared_ptr<WorkItem> _w;

	static void callbackFunc(void *arg)
	{
		std::cout << "In callback\n";

		// Reschedule the work
		HRTWorkerThread *wt = HRTWorkerThread::instance();
		Worker *me = (Worker *)arg;
		wt->scheduleWorkItem(me->_w);
	}
};

int main()
{
	HRTWorkerThread *wt = HRTWorkerThread::instance();

	wt->init();

	Worker wrkr;

	wrkr.doWork();

	wt->cleanup();

	return 0;
}
