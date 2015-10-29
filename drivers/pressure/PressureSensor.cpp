/****************************************************************************
 *
 *   Copyright (C) 2015 Mark Charlebois. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <pthread.h>
#include "DriverFramework.hpp"
#include "PressureSensor.hpp"

using namespace DriverFramework;


int PressureSensor::start()
{
	int ret = I2CDriverObj::start();
	if (ret != 0) {
		return ret;
	}

	// Start polling the sensor
	m_work_handle = WorkItemMgr::create(workCallback, this, m_sample_interval);
	WorkItemMgr::schedule(m_work_handle);

	return ret;
}

int PressureSensor::stop()
{
	WorkItemMgr::destroy(m_work_handle);
	return I2CDriverObj::stop();
}

void PressureSensor::setAltimeter(float altimeter_setting_in_mbars)
{
	m_altimeter_mbars = altimeter_setting_in_mbars;
}

int PressureSensor::getSensorData(struct pressure_sensor_data &out_data, bool is_new_data_required)
{
	m_synchronize.lock();
	if (is_new_data_required) {
		m_synchronize.waitOnSignal();
	}
	out_data = m_sensor_data;
	m_synchronize.unlock();

	return 1;
}

uint32_t PressureSensor::getPressure()
{
	// TBD
	return 0;
}

float PressureSensor::getTemperature()
{
	// TBD
	return 0.0;
}

void PressureSensor::workCallback(void *arg, WorkHandle wh)
{
	PressureSensor *me = (PressureSensor *)arg;

	me->readSensor();
	WorkItemMgr::schedule(wh);
}

void PressureSensor::readSensor(void)
{
	m_synchronize.lock();

	m_sensor_data.pressure_in_pa = getPressure();
	m_sensor_data.temperature_in_c = getTemperature();
	m_sensor_data.last_read_time_in_usecs = DriverFramework::offsetTime();
	m_sensor_data.sensor_read_counter++;

	m_synchronize.signal();
	m_synchronize.unlock();
}
