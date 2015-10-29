#include <stdio.h>
#include <unistd.h>
#include <unistd.h>
#include "DriverFramework.hpp"
#include "PressureSensor.hpp"

using namespace DriverFramework;

class PressureTester
{
public:
	PressureTester() :
		m_sensor(PRESSURE_DEVICE_PATH)
	{}

	static void readSensorCallback(void *arg, Workhandle wh);

	int run(void);

private:
	void readSensor(Workhandle wh);
	void wait();

	PressureSensor	m_sensor;
	WorkHandle	m_work = 0;
	uint32_t 	m_read_attempts = 0;
	uint32_t 	m_read_counter = 0;
	bool		m_result = false;
	struct pressure_sensor_data m_sensor_data;
};
	
void PressureTester::readSensorCallback(void *arg, Workhandle wh)
{
	PressureTester *me = (PressureTester *)arg;
	me->readSensor(Workhandle wh);
}

void PressureTester::wait(void)
{
	while(!m_done) {
		sleep(1);
	}
}

void PressureTester::readSensor(Workhandle wh)
{
	status = m_sensor.getSensorData(m_sensor_data, true);

	if ((status == 0) && (m_read_counter != m_sensor_data.sensor_read_counter)) {
		m_read_counter = m_sensor_data.sensor_read_counter;
		printPressureValues();
	} else {
		printf("error: unable to read the pressure sensor device.");
	}
	if ((m_read_counter < 1000) && (m_read_attempts < 2000)) {
		WorkItemMgr::schedule(wh);
	}
	else {
		m_done = true;
	}
}

int PressureTester::run(void) {
	int status = 0;

	print("Entering: run");

	// Open the pressure sensor
	int ret = m_sensor.open();
	if (ret != 0)
	{
		printf("Error: unable to obtain a valid handle for the receiver at: %s"
			PRESSURE_DEVICE_PATH);
		m_pass = TEST_FAIL;
	}
	else {
		m_done = false;
		m_sensor_data.sensor_read_counter = 0;
		m_work_handle = WorkItemMgr::create(readSensorCallback, this, 1000);

		WorkItemMgr::schedule(m_work_handle);

		wait();
		m_pass = TEST_PASS;
	}

	print("Closing pressure sensor\n");
	m_sensor.close();
	return m_pass;
}

int main()
{
	int ret = DriverFramework::initialize();
	if (ret < 0) {
		return ret;
	}

	PressureTester pt;

	ret = pt.run;

	// No need to block the framework here
	//DriverFramework::waitForShutdown();

	DriverFramework::shutdown();

	return ret;
}
