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

	static void readSensorCallback(void *arg);

	int run(void);

private:
	void readSensor();
	void wait();

	PressureSensor	m_sensor;
	uint32_t 	m_read_attempts = 0;
	uint32_t 	m_read_counter = 0;
	struct pressure_sensor_data m_sensor_data;

	int		m_pass;
	bool		m_done = false;
};

static void printPressureValues(struct pressure_sensor_data &sensor_data)
{
	DF_LOG_INFO("bmp280 data [cntr: %" PRIu32 ", time stamp: %" PRId64 ", pressure (pascals): %" PRIu32 ", temp (C): %f]",
        	sensor_data.sensor_read_counter, sensor_data.last_read_time_in_usecs, 
		sensor_data.pressure_in_pa, sensor_data.temperature_in_c);
}

int PressureTester::run() 
{
	DF_LOG_INFO("Entering: run");
	// Default is fail unless pass critera met
	m_pass = TEST_FAIL;

	// Register the driver
	int ret = m_sensor.init();

	// Open the pressure sensor
	DevHandle h;
	DevMgr::getHandle(PRESSURE_DEVICE_PATH, h);
	if (!h.isValid())
	{
		DF_LOG_INFO("Error: unable to obtain a valid handle for the receiver at: %s (%d)", 
			PRESSURE_DEVICE_PATH, h.getError());
		m_done = true;
	}
	else {
		m_done = false;
		m_sensor_data.sensor_read_counter = 0;
	}

	while (!m_done) {
		++m_read_attempts;
		ret = PressureSensor::getSensorData(h, m_sensor_data, true);
		if (ret == 0) {
			uint32_t count = m_sensor_data.sensor_read_counter;
			if (m_read_counter != count) {
				m_read_counter = count;
				printPressureValues(m_sensor_data);
			}
		} 
		else {
			DF_LOG_INFO("error: unable to read the pressure sensor device.");
		}

		if ((m_read_counter >= 1000) && (m_read_attempts == m_read_counter)) {
			// Done test - PASSED
			m_pass = TEST_PASS;
			m_done = true;
		}
		else if (m_read_attempts > 1000) {
			DF_LOG_INFO("error: unable to read the pressure sensor device.");
			m_done = true;
		}
	}

	DF_LOG_INFO("Closing pressure sensor\n");
	m_sensor.stop();
	return m_pass;
}

int main()
{
	int ret = Framework::initialize();
	if (ret < 0) {
		return ret;
	}

	PressureTester pt;

	ret = pt.run();

	Framework::shutdown();

	DF_LOG_INFO("Test %s", (ret == PressureTester::TEST_PASS) ? "PASSED" : "FAILED");
	return ret;
}

