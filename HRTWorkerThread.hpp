/**********************************************************************
* Copyright (c) 2015 Mark Charlebois
* 
* All rights reserved.
* 
* Redistribution and use in source and binary forms, with or without
* modification, are permitted (subject to the limitations in the
* disclaimer below) provided that the following conditions are met:
* 
*  * Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
* 
*  * Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the
*    distribution.
* 
*  * Neither the name of Dronecode Project nor the names of its
*    contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
* 
* NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
* GRANTED BY THIS LICENSE.  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
* HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
* WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
* MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
* BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
* WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
* OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
* IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*************************************************************************/
#include<memory>
#include<map>
#include<list>
#include<string>
#include<stdint.h>

uint64_t HRTAbsoluteTime();

class WorkItem;

typedef void (*workCallback)(void *arg);

class WorkItem
{
public:
	WorkItem(workCallback callback, void *arg, uint32_t delay) : 
		m_arg(arg),
		m_queue_time(0),
		m_callback(callback),
		m_delay(delay)
	{}
	~WorkItem() {}

	void *		m_arg;
	uint64_t	m_queue_time;
	workCallback	m_callback;
	uint32_t	m_delay;
};

class WorkItemStats
{
public:
	WorkItemStats() 
	{
		reset();
	}
	
	~WorkItemStats() {};

	void update(unsigned int cur_usec);

	void reset();

	unsigned long m_last;
	unsigned long m_min;
	unsigned long m_max;
	unsigned long m_total;
	unsigned long m_count;
};

class HRTWorkerThread
{
public:
	static HRTWorkerThread *instance(void);

	int init(void);
	void cleanup(void);

	void scheduleWorkItem(std::shared_ptr<WorkItem> item);

	std::map<workCallback, WorkItemStats> getStats();

	void enableStats(bool enable);

	static void *process_trampoline(void *);

private:
	HRTWorkerThread(void) {}
	~HRTWorkerThread(void) {}

	void process(void);

	void hrtLock(void);
	void hrtUnlock(void);

	std::list<std::shared_ptr<WorkItem>>	m_work;
	std::map<workCallback, WorkItemStats>	m_stats;

	bool m_enable_stats = false;
};

