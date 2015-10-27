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

	static void doWork()
	{
		HRTWorkerThread *wt = HRTWorkerThread::instance();
		wt->scheduleWorkItem(_w);
	}

	static void *worker_main(void *arg)
	{
		g_instance = new Worker();

		g_instance->doWork();

		return NULL;
	}

private:
	static std::shared_ptr<WorkItem> _w;

	static void callbackFunc(void *arg)
	{
		std::cout << "In callback\n";

		// Reschedule the work
		HRTWorkerThread *wt = HRTWorkerThread::instance();
		Worker *me = (Worker *)arg;
		wt->scheduleWorkItem(me->_w);
	}
	static Worker *g_instance;
};

std::shared_ptr<WorkItem> Worker::_w;

Worker *Worker::g_instance = nullptr;

int main()
{
	pthread_t tid;
	HRTWorkerThread *wt = HRTWorkerThread::instance();

	wt->init();

	int ret = pthread_create(&tid, NULL, Worker::worker_main, NULL);
	void *val;

	pthread_join(tid, &val);
	wt->cleanup();

	return 0;
}
