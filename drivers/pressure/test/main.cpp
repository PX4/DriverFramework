#define __STDC_FORMAT_MACROS
#include <inttypes.h>
#include <stdio.h>
#include <unistd.h>
#include <unistd.h>
#include "DriverFramework.hpp"
#include "PressureSensor.hpp"

using namespace DriverFramework;

class PressureTester
{
public:

	static const int TEST_PASS = 0;
	static const int TEST_FAIL = 1;

	PressureTester() :
		m_sensor(PRESSURE_DEVICE_PATH)
	{}

	static void readSensorCallback(void *arg, WorkHandle wh);

	int run(void);

private:
	void readSensor();
	void wait();

	PressureSensor	m_sensor;
	WorkHandle	m_work_handle = 0;
	uint32_t 	m_read_attempts = 0;
	uint32_t 	m_read_counter = 0;
	struct pressure_sensor_data m_sensor_data;

	int		m_pass;
	bool		m_done = false;
};

static void printPressureValues(struct pressure_sensor_data &sensor_data)
{
	DF_LOG_INFO("bmp280 data [cntr: %d, time stamp: %" PRId64 ", pressure (pascals): %d, temp (C): %f]",
        	sensor_data.sensor_read_counter, sensor_data.last_read_time_in_usecs, 
		sensor_data.pressure_in_pa, sensor_data.temperature_in_c);
}

void PressureTester::readSensorCallback(void *arg, WorkHandle wh)
{
	PressureTester *me = (PressureTester *)arg;
	me->readSensor();
}

void PressureTester::wait(void)
{
	// Use a simple block for the test
	while(!m_done) {
		sleep(1);
	}
}

void PressureTester::readSensor()
{
	int status = m_sensor.getSensorData(m_sensor_data, true);

	if ((status == 0) && (m_read_counter != m_sensor_data.sensor_read_counter)) {
		m_read_counter = m_sensor_data.sensor_read_counter;
		printPressureValues(m_sensor_data);
	} else {
		DF_LOG_INFO("error: unable to read the pressure sensor device.");
	}
	if ((m_read_counter < 1000) && (m_read_attempts < 2000)) {
		WorkItemMgr::schedule(m_work_handle);
	}
	else {
		// Done test
		m_pass = TEST_PASS;
		m_done = true;
	}
}

int PressureTester::run(void) {
	DF_LOG_INFO("Entering: run");

	// Open the pressure sensor
	int ret = m_sensor.start();
	if (ret != 0)
	{
		DF_LOG_INFO("Error: unable to obtain a valid handle for the receiver at: %s", 
			PRESSURE_DEVICE_PATH);
		m_pass = TEST_FAIL;
		m_done = true;
	}
	else {
		m_done = false;
		m_sensor_data.sensor_read_counter = 0;
		m_work_handle = WorkItemMgr::create(readSensorCallback, this, 1000);

		WorkItemMgr::schedule(m_work_handle);
	}

	wait();
	DF_LOG_INFO("Closing pressure sensor\n");
	m_sensor.stop();
	return m_pass;
}

int main()
{
	int ret = DriverFramework::initialize();
	if (ret < 0) {
		return ret;
	}

	PressureTester pt;

	ret = pt.run();

	DriverFramework::shutdown();

	DF_LOG_INFO("Test %s", (ret == PressureTester::TEST_PASS) ? "PASSED" : "FAILED");
	return ret;
}
