/****************************************************************************
 *
 *   Copyright (C) 2015 Mark Charlebois. All rights reserved.
 *   Copyright (C) 2016 PX4 Development Team. All rights reserved.
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

#include "DriverFramework.hpp"
#include "MS5611.hpp"

#include <string.h>
#include <signal.h>
#include <asm/unistd.h>

using namespace DriverFramework;

#define ADDR_RESET_CMD				0x1E	/* write to this address to reset chip */
#define ADDR_CMD_CONVERT_D1_OSR256		0x40	/* write to this address to start pressure conversion */
#define ADDR_CMD_CONVERT_D1_OSR512		0x42	/* write to this address to start pressure conversion */
#define ADDR_CMD_CONVERT_D1_OSR1024		0x44	/* write to this address to start pressure conversion */
#define ADDR_CMD_CONVERT_D1_OSR2048		0x46	/* write to this address to start pressure conversion */
#define ADDR_CMD_CONVERT_D1_OSR4096		0x48	/* write to this address to start pressure conversion */
#define ADDR_CMD_CONVERT_D2_OSR256		0x50	/* write to this address to start temperature conversion */
#define ADDR_CMD_CONVERT_D2_OSR512		0x52	/* write to this address to start temperature conversion */
#define ADDR_CMD_CONVERT_D2_OSR1024		0x54	/* write to this address to start temperature conversion */
#define ADDR_CMD_CONVERT_D2_OSR2048		0x56	/* write to this address to start temperature conversion */
#define ADDR_CMD_CONVERT_D2_OSR4096		0x58	/* write to this address to start temperature conversion */

#define MS5611_CONVERT_TIME_MAX_OSR1024_US (2280) /* 2.28 ms */
#define MS5611_CONVERT_TIME_MAX_OSR2048_US (4540) /* 4.54 ms */

#define MS5611_CONVERT_TIME_MIN_OSR4096_US (7400) /* 7.40 ms */
#define MS5611_CONVERT_TIME_TYP_OSR4096_US (8220) /* 8.22 ms */
#define MS5611_CONVERT_TIME_MAX_OSR4096_US (9040) /* 9.04 ms */


#define MS5611_SAMPLES_PER_TEMPERATURE (1)

#define ADDR_CMD_CONVERT_D1   ADDR_CMD_CONVERT_D1_OSR4096
#define ADDR_CMD_CONVERT_D2   ADDR_CMD_CONVERT_D2_OSR4096
#define MS5611_CONVERT_TIME_US (MS5611_CONVERT_TIME_TYP_OSR4096_US)

#define ADDR_CMD_ADC_READ     0x00
#define ADDR_PROM_SETUP       0xA0  /* address of 8x 2 bytes factory and calibration data */

#define POW2(_x) ((_x) * (_x))

/* #define MS5611_DEBUG_TIMING 1 */

static void timespec_inc(struct timespec *timespec, long dt)
{
	timespec->tv_nsec += dt;

	while (timespec->tv_nsec >= 1000000000) {
		/* timespec nsec overflow */
		timespec->tv_sec++;
		timespec->tv_nsec -= 1000000000;
	}
}

MS5611::MS5611(const char *device_path)
	: BaroSensor(device_path, 0)
	, _thread_id(0)
	, _started(false)
{
	m_id.dev_id_s.devtype = DRV_DF_DEVTYPE_MS5611;
	m_id.dev_id_s.address = MS5611_I2C_ADDR;
}

// convertPressure must be called after convertTemperature
// as convertTemperature sets m_raw_sensor_convertion values
int64_t MS5611::convertPressure(int64_t adc_P)
{
	// Conversion from the datasheet
	int64_t p = (((adc_P * m_raw_sensor_convertion.sens) >> 21) - m_raw_sensor_convertion.off) >> 15;
	m_raw_sensor_convertion.pressure_mbar = p;
	return p;
}

int32_t MS5611::convertTemperature(int32_t adc_T)
{
	// Conversion from the datasheet
	/* temperature offset (in ADC units) */
	int32_t dT = adc_T - ((int32_t)m_sensor_calibration.c5_reference_temp << 8);

	int64_t sens = ((int64_t)m_sensor_calibration.c1_pressure_sens << 15)
		       + (((int64_t)m_sensor_calibration.c3_temp_coeff_pres_sens * dT) >> 8);

	int64_t off = ((int64_t)m_sensor_calibration.c2_pressure_offset << 16)
		      + (((int64_t)m_sensor_calibration.c4_temp_coeff_pres_offset * dT) >> 7);

	/* absolute temperature in centidegrees - note intermediate value is outside 32-bit range */
	int32_t temp =  2000 + (int32_t)(((int64_t)dT * m_sensor_calibration.c6_temp_coeff_temp) >> 23);

	/* temperature compensation */
	if (temp < 2000) {
		int32_t t2 = POW2(dT) >> 31;

		int64_t f = POW2((int64_t)temp - 2000);
		int64_t off2 = 5 * f >> 1;
		int64_t sens2 = 5 * f >> 2;

		if (temp < -1500) {
			int64_t f2 = POW2(temp + 1500);
			off2 += 7 * f2;
			sens2 += 11 * f2 >> 2;
		}

		temp -= t2;
		sens -= sens2;
		off -= off2;
	}

	m_raw_sensor_convertion.temperature_cc = temp;
	m_raw_sensor_convertion.sens = sens;
	m_raw_sensor_convertion.off = off;
	return temp;
}

/**
* MS5611 crc4 cribbed from the datasheet
*/
bool MS5611::crc4(uint16_t *n_prom)
{
	int16_t cnt;
	uint16_t n_rem;
	uint16_t crc_read;
	uint8_t n_bit;

	n_rem = 0x00;

	/* save the read crc */
	crc_read = n_prom[7];

	/* remove CRC byte */
	n_prom[7] = (0xFF00 & (n_prom[7]));

	for (cnt = 0; cnt < 16; cnt++) {
		/* uneven bytes */
		if (cnt & 1) {
			n_rem ^= (uint8_t)((n_prom[cnt >> 1]) & 0x00FF);

		} else {
			n_rem ^= (uint8_t)(n_prom[cnt >> 1] >> 8);
		}

		for (n_bit = 8; n_bit > 0; n_bit--) {
			if (n_rem & 0x8000) {
				n_rem = (n_rem << 1) ^ 0x3000;

			} else {
				n_rem = (n_rem << 1);
			}
		}
	}

	/* final 4 bit remainder is CRC value */
	n_rem = (0x000F & (n_rem >> 12));
	n_prom[7] = crc_read;

	/* return true if CRCs match */
	return (0x000F & crc_read) == (n_rem ^ 0x00);
}


int MS5611::loadCalibration()
{
	// Wait for PROM contents to be in the device (2.8 ms), in case we are called
	// immediatelly after reset.
	usleep(3000);

	uint8_t last_val = 0;
	bool bits_stuck = true;

	uint8_t prom_buf[2];
	union {
		uint8_t b[2];
		uint16_t w;
	} cvt {};

	for (int i = 0; i < 8; ++i) {
		uint8_t cmd = ADDR_PROM_SETUP + (i * 2);

#if defined(__BARO_USE_SPI)

		if (_bulkRead(cmd, &prom_buf[0], 2) < 0) {
#else
		_retries = 5;

		if (_readReg(cmd, &prom_buf[0], 2) < 0) {
#endif
			DF_LOG_ERR("Read calibration error");
			break;
		}

		// check if all bytes are zero
		if (i == 0) {
			last_val = prom_buf[0];
		}

		if (prom_buf[0] != last_val || prom_buf[1] != last_val) {
			bits_stuck = false;
		}

		cvt.b[0] = prom_buf[1];
		cvt.b[1] = prom_buf[0];
		memcpy(((uint16_t *)&m_sensor_calibration + i), &cvt.w, sizeof(uint16_t));
	}

	DF_LOG_DEBUG("factory_setup: %d", m_sensor_calibration.factory_setup);
	DF_LOG_DEBUG("c1: %d", m_sensor_calibration.c1_pressure_sens);
	DF_LOG_DEBUG("c2: %d", m_sensor_calibration.c2_pressure_offset);
	DF_LOG_DEBUG("c3: %d", m_sensor_calibration.c3_temp_coeff_pres_sens);
	DF_LOG_DEBUG("c4: %d", m_sensor_calibration.c4_temp_coeff_pres_offset);
	DF_LOG_DEBUG("c5: %d", m_sensor_calibration.c5_reference_temp);
	DF_LOG_DEBUG("c6: %d", m_sensor_calibration.c6_temp_coeff_temp);

	return (crc4((uint16_t *)&m_sensor_calibration) && !bits_stuck) ? 0 : -1;
}

int MS5611::reset()
{
	int result;
	uint8_t cmd = ADDR_RESET_CMD;

#if defined(__BARO_USE_SPI)
	uint8_t wbuf[1];
	uint8_t rbuf[1];
	wbuf[0] = cmd;
	result = _transfer(wbuf, rbuf, 1);

#else

	_retries = 10;
	result = _writeReg(cmd, nullptr, 0);

#endif

	if (result < 0) {
		DF_LOG_ERR("Unable to reset device: %d", result);
		return -EIO;
	}

	return result;
}

int MS5611::start()
{
	pthread_attr_t attr;
	struct sched_param param = {};

	int result;

	if (_started) {
		DF_LOG_ERR("MS5611 already started");
		return 0;
	}

	_started = true;

#if defined(__BARO_USE_SPI)
	result = SPIDevObj::start();
#else
	result = I2CDevObj::start();
#endif

	if (result != 0) {
		DF_LOG_ERR("error: could not start DevObj");
		goto exit;
	}

	/* Zero the struct */

	m_sensor_data.pressure_pa = 0.0f;
	m_sensor_data.temperature_c = 0.0f;
	m_sensor_data.last_read_time_usec = 0;
	m_sensor_data.read_counter = 0;
	m_sensor_data.error_counter = 0;

#if defined(__BARO_USE_SPI)
	result = _setBusFrequency(MS5611_SPI_FREQ_HZ);
#else
	result = _setSlaveConfig(MS5611_I2C_ADDR,
				     MS5611_I2C_FREQ_KHZ,
				     MS5611_I2C_TIMEOUT_US);
#endif

	if (result < 0) {
		DF_LOG_ERR("could not set slave config");
	}

	/* Reset sensor and load calibration data into internal register */
	result = reset();

	if (result < 0) {
		DF_LOG_ERR("error: unable to communicate with the MS5611 pressure sensor");
		return -EIO;
	}

	result = loadCalibration();

	if (result != 0) {
		DF_LOG_ERR("error: unable to complete initialization of the MS5611 pressure sensor");
		return -EIO;
	}

	result = DevObj::start();

	if (result != 0) {
		DF_LOG_ERR("error: could not start DevObj");
		goto exit;
	}

	result = pthread_attr_init(&attr);

	if (result != 0) {
		DF_LOG_ERR("pthread_attr_init: %d", result);
		goto exit;
	}

	result = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);

	if (result != 0) {
		DF_LOG_ERR("pthread_attr_setinheritsched: %d", result);
		goto exit;
	}

	result = pthread_attr_setschedpolicy(&attr, SCHED_FIFO);

	if (result != 0) {
		DF_LOG_ERR("pthread_attr_setschedpolicy: %d", result);
		goto exit;
	}

	param.sched_priority = sched_get_priority_max(SCHED_FIFO) - 1;
	result = pthread_attr_setschedparam(&attr, &param);

	if (result != 0) {
		DF_LOG_ERR("pthread_attr_setschedparam: %d", result);
		goto exit;
	}

	result = pthread_create(&_thread_id, &attr, MS5611::threadFunc, this);

	if (result != 0) {
		if (result == EPERM) {
			DF_LOG_ERR("pthread_create: %d; run as root!", result);
			goto exit;
		} else {
			DF_LOG_ERR("pthread_create: %d", result);
			goto exit;
		}
	}

	pthread_attr_destroy(&attr);

exit:

	return result;
}

int MS5611::stop()
{
	void *retval;
	int result = DevObj::stop();

	if (result != 0) {
		DF_LOG_ERR("DevObj stop failed");
		return result;
	}

	result = pthread_kill(_thread_id, SIGUSR1);
	if (result != 0) {
		DF_LOG_ERR("pthread_kill: %d", result);
		return result;
	}

	result = pthread_join(_thread_id, &retval);
	if (result != 0) {
		DF_LOG_ERR("pthread_join: %d", result);
	}

	return 0;
}

int MS5611::_request(uint8_t cmd)
{
	int ret;

#if defined(__BARO_USE_SPI)
	uint8_t wbuf[1];
	uint8_t rbuf[1];

	wbuf[0] = cmd;
	ret = _transfer(wbuf, rbuf, 1);
#else
	_retries = 0;
	ret = _writeReg(cmd, nullptr, 0);
#endif

	if (ret < 0) {
		DF_LOG_ERR("error: request failed");
	}

	return ret;
}

int MS5611::_collect(uint32_t &raw)
{
	int ret;

	union {
		uint8_t b[4];
		uint32_t w;
	} cvt {};

	uint8_t cmd = ADDR_CMD_ADC_READ;

#if defined(__BARO_USE_SPI)
	uint8_t buf[4];
	uint8_t wbuf[4];
	wbuf[0] = cmd;

	ret = _transfer(&wbuf[0], &buf[0], 4);

	if (ret < 0) {
		raw = 0;
		return -1;
	}

	cvt.b[0] = buf[3];
	cvt.b[1] = buf[2];
	cvt.b[2] = buf[1];
	cvt.b[3] = 0;
	raw = cvt.w;

	return 0;

#else
	uint8_t buf[3];
	_retries = 0;

	ret = _readReg(cmd, &buf[0], 3);

	if (ret < 0) {
		raw = 0;
		return -1;
	}

	cvt.b[0] = buf[2];
	cvt.b[1] = buf[1];
	cvt.b[2] = buf[0];
	cvt.b[3] = 0;
	raw = cvt.w;

	return 0;
#endif
}

void MS5611::_measure()
{
	struct timespec next_wakeup;
	int ret;

	uint32_t temperature_from_sensor;
	uint32_t pressure_from_sensor;

	if (_request(ADDR_CMD_CONVERT_D2) < 0) {
		DF_LOG_ERR("error: temp measure failed");
		return;
	}

	clock_gettime(CLOCK_MONOTONIC, &next_wakeup);
	timespec_inc(&next_wakeup, MS5611_CONVERT_TIME_US * 1000);
	ret = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_wakeup, NULL);
	if (ret == EINTR) {
		DF_LOG_ERR("MS5611 (%d) someone sent me a signal", __LINE__);
		return;
	} else if (ret != 0) {
		DF_LOG_ERR("MS5611 (%d) failed to sleep: %d", __LINE__, ret);
		return;
	}

	if (_collect(temperature_from_sensor) < 0) {
		DF_LOG_ERR("error: temp collect failed");
		reset();
		return;
	}

	if (temperature_from_sensor == 0) {
		DF_LOG_ERR("MS5611: invalid temperature sample!");
		return;
	}

	for (unsigned int i = 0; i < MS5611_SAMPLES_PER_TEMPERATURE; i++) {
		// Request to convert the pressure
		if (_request(ADDR_CMD_CONVERT_D1) < 0) {
			DF_LOG_ERR("error: pressure measure failed");
		}

		clock_gettime(CLOCK_MONOTONIC, &next_wakeup);
		timespec_inc(&next_wakeup, MS5611_CONVERT_TIME_US * 1000);
		ret = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_wakeup, NULL);
		if (ret == EINTR) {
			DF_LOG_ERR("MS5611 (%d) someone sent me a signal", __LINE__);
			return;
		} else if (ret != 0) {
			DF_LOG_ERR("MS5611 (%d) failed to sleep: %d", __LINE__, ret);
			return;
		}

		if (_collect(pressure_from_sensor) < 0) {
			DF_LOG_ERR("error: pressure collect failed");
			reset();

			return;
		}

		if (pressure_from_sensor == 0) {
			DF_LOG_ERR("MS5611: invalid presure sample!");
			return;
		}

		// Request to convert the temperature
		if (_request(ADDR_CMD_CONVERT_D2) < 0) {
			DF_LOG_ERR("error: temp measure failed");
		}

		m_sensor_data.temperature_c = convertTemperature(temperature_from_sensor) / 100.0;
		m_sensor_data.pressure_pa = convertPressure(pressure_from_sensor);
		m_sensor_data.last_read_time_usec = DriverFramework::offsetTime();
		m_sensor_data.read_counter++;

		_publish(m_sensor_data);
	}
}

#ifdef MS5611_DEBUG_TIMING
static long int timespec_diff(struct timespec *start, struct timespec *stop)
{
	long int diff = (stop->tv_sec - start->tv_sec) * 1000000000LL + (stop->tv_nsec - start->tv_nsec);
	return diff;
}
#endif

void* MS5611::threadFunc(void *arg)
{
	MS5611 *instance = static_cast<MS5611*>(arg);

	long wakeup_period_ns = MS5611_MEASURE_INTERVAL_US * 1000;
	struct timespec next_wakeup;
#ifdef MS5611_DEBUG_TIMING
	struct timespec start_time;
	struct timespec end_time;
	struct timespec sleep_time;
	long int sleep_dt;
	long int exec_dt;
	double sleep_dt_mean = 0;
	double exec_dt_mean = 0;
	int stats_print_cnt = 0;

	clock_gettime(CLOCK_MONOTONIC, &sleep_time);
#endif

	DF_LOG_ERR("MS5611 TID: %d", (long int)syscall(__NR_gettid));

	clock_gettime(CLOCK_MONOTONIC, &next_wakeup);
	while (1) {
#ifdef MS5611_DEBUG_TIMING
		clock_gettime(CLOCK_MONOTONIC, &start_time);
#endif
		instance->_measure();
#ifdef MS5611_DEBUG_TIMING
		clock_gettime(CLOCK_MONOTONIC, &end_time);
#endif

#ifdef MS5611_DEBUG_TIMING
		sleep_dt = timespec_diff(&sleep_time, &start_time);
		exec_dt = timespec_diff(&start_time, &end_time);
		sleep_dt_mean = (sleep_dt_mean + sleep_dt) / 2.0;
		exec_dt_mean = (exec_dt_mean + exec_dt) / 2.0;

		if (stats_print_cnt / 100 != 0) {
			printf("MS5611 sleep_dt_mean: %f\n", sleep_dt_mean);
			printf("MS5611 exec_dt_mean: %f\n", exec_dt_mean);
			stats_print_cnt = 0;
		} else {
			stats_print_cnt++;
		}
#endif

		timespec_inc(&next_wakeup, wakeup_period_ns);

#ifdef MS5611_DEBUG_TIMING
		clock_gettime(CLOCK_MONOTONIC, &sleep_time);
#endif
		int ret = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_wakeup, NULL);
		if (ret == EINTR) {
			DF_LOG_ERR("MS5611 someone sent me a signal");
			return NULL;
		} else if (ret != 0) {
			DF_LOG_ERR("MS5611 fail to sleep: %d", ret);
			return NULL;
		}
	}

	return NULL;
}

