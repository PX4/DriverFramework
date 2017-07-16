/****************************************************************************
 *
 *   Copyright (C) 2016 Julian Oes. All rights reserved.
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

#include <stdint.h>
#include <string.h>
#include <signal.h>
#include <byteswap.h>

#include "math.h"
#include "DriverFramework.hpp"
#include "MPU9250.hpp"
#include "MPU9250_mag.hpp"

#define MPU9250_ONE_G	9.80665f

#define MPU9250_CHECK_DUPLICATES (0)

#define MIN(_x, _y) (_x) > (_y) ? (_y) : (_x)

#define MPU9250_DEBUG_TIMING (0)

using namespace DriverFramework;

MPU9250::MPU9250(const char *device_path, bool mag_enabled) :
	ImuSensor(device_path, 0, mag_enabled), // true = mag is enabled
	_last_temp_c(0.0f),
	_temp_initialized(false),
	_mag_enabled(mag_enabled),
	_mag(nullptr)
	, _started(false)
{
	m_id.dev_id_s.devtype = DRV_DF_DEVTYPE_MPU9250;
	// TODO: does the WHOAMI make sense as an address?
	m_id.dev_id_s.address = MPU_WHOAMI_9250;

	_counters.fifo_avg_packets = MPU9250_PACKETS_PER_CYCLE;
}

int MPU9250::writeReg_checked(uint8_t addr, uint8_t val)
{
	uint8_t val_out;
	int result;

	result = _writeReg(addr, val);

	if (result != 0) {
		DF_LOG_ERR("%d: write failed", (int)addr);
		return -1;
	}

	result = _readReg(addr, val_out);

	if (result != 0) {
		DF_LOG_ERR("%d: read failed", (int)addr);
		return -1;
	}

	if (val != val_out) {
		DF_LOG_ERR("%d: set failed", (int)addr);
		return -1;
	}

	return 0;
}

int MPU9250::mpu9250_init()
{
	int result;
	// Use 1 MHz for normal registers.
	result = _setBusFrequency(SPI_FREQUENCY_1MHZ);
	if (result) {
		DF_LOG_ERR("_setBusFrequency failed: %d\n", result);
	}

	_counters.accel_range_hits = 0;
	_counters.gyro_range_hits = 0;

#if MPU9250_CHECK_DUPLICATES
	_counters.accel_duplicates = 0;
	_counters.gyro_duplicates = 0;
	_counters.mag_duplicates = 0;
#endif

	_counters.mag_overflows = 0;
	_counters.mag_overruns = 0;

	_counters.fifo_overflows = 0;
	_counters.fifo_reads = 0;
	_counters.fifo_corruptions = 0;

	_counters.errors = 0;

	result = _writeReg(MPUREG_PWR_MGMT_1, BIT_H_RESET);

	if (result != 0) {
		DF_LOG_ERR("reset failed");
	}

	usleep(100000);

	DF_LOG_INFO("Reset MPU9250");
	result = writeReg_checked(MPUREG_PWR_MGMT_1, 0);

	if (result != 0) {
		DF_LOG_ERR("wakeup sensor failed");
	}

	usleep(1000);

	result = writeReg_checked(MPUREG_PWR_MGMT_2, 0);

	if (result != 0) {
		DF_LOG_ERR("enable failed");
	}

	usleep(1000);

	// Reset I2C master and device.
	result = writeReg(MPUREG_USER_CTRL,
			   BITS_USER_CTRL_I2C_MST_RST |
			   BITS_USER_CTRL_I2C_IF_DIS);

	if (result != 0) {
		DF_LOG_ERR("user ctrl 1 failed");
	}

	usleep(1000);

	// Reset and enable FIFO.
	result = writeReg(MPUREG_USER_CTRL,
			   BITS_USER_CTRL_FIFO_RST |
			   BITS_USER_CTRL_FIFO_EN);

	if (result != 0) {
		DF_LOG_ERR("user ctrl 2 failed");
	}

	usleep(1000);

	if (_mag_enabled) {
		result = _writeReg(MPUREG_FIFO_EN,
				   BITS_FIFO_ENABLE_TEMP_OUT | BITS_FIFO_ENABLE_GYRO_XOUT
				   | BITS_FIFO_ENABLE_GYRO_YOUT
				   | BITS_FIFO_ENABLE_GYRO_ZOUT | BITS_FIFO_ENABLE_ACCEL
				   | BITS_FIFO_ENABLE_SLV0); // SLV0 is configured for bulk transfer of mag data over I2C

	} else {
		result = _writeReg(MPUREG_FIFO_EN,
				   BITS_FIFO_ENABLE_TEMP_OUT | BITS_FIFO_ENABLE_GYRO_XOUT
				   | BITS_FIFO_ENABLE_GYRO_YOUT
				   | BITS_FIFO_ENABLE_GYRO_ZOUT | BITS_FIFO_ENABLE_ACCEL);
		DF_LOG_INFO("initializing mpu9250 driver without mag support");
	}

	if (result != 0) {
		DF_LOG_ERR("FIFO enable failed");
	}

	usleep(1000);

	uint8_t regValue;
	regValue = BITS_DLPF_CFG_184HZ | BITS_CONFIG_FIFO_MODE_OVERWRITE;
	result = writeReg_checked(MPUREG_CONFIG, regValue);

	if (result != 0) {
		DF_LOG_ERR("MPUREG_CONFIG: write failed");
		return -1;
	}

	usleep(1000);

	regValue = BITS_FS_2000DPS | BITS_BW_LT3600HZ;
	result = writeReg_checked(MPUREG_GYRO_CONFIG, regValue);

	if (result != 0) {
		DF_LOG_ERR("MPUREG_GYRO_CONFIG: write failed");
		return -1;
	}

	usleep(1000);

	/* Accel config */
	regValue = BITS_ACCEL_CONFIG_16G;
	result = writeReg_checked(MPUREG_ACCEL_CONFIG, regValue);

	if (result != 0) {
		DF_LOG_ERR("MPUREG_ACCEL_CONFIG: write failed");
	}

	usleep(1000);

	regValue = BITS_ACCEL_CONFIG2_BW_218HZ;
	result = writeReg_checked(MPUREG_ACCEL_CONFIG2, regValue);

	if (result != 0) {
		DF_LOG_ERR("MPUREG_ACCEL_CONFIG2: write failed");
	}

	usleep(1000);

	// Initialize the magnetometer inside the IMU, if enabled by the caller.
	if (_mag_enabled && _mag == nullptr) {
		if ((_mag = new MPU9250_mag(*this, MPU9250_MAG_SAMPLE_RATE_100HZ))
		    != nullptr) {
			// Initialize the magnetometer, providing the output data rate for
			// data read from the IMU FIFO.  This is used to calculate the I2C
			// delay for reading the magnetometer.
			result = _mag->initialize(MPU9250_MEASURE_INTERVAL_US);

			if (result != 0) {
				DF_LOG_ERR("Magnetometer initialization failed");
			}

		} else {
			DF_LOG_ERR("Allocation of magnetometer object failed.");
		}
	}

	// Enable/clear the FIFO of any residual data
	reset_fifo();

	// Clear Interrupt Status
	clear_int_status();

	if (_mag_enabled) {
		_size_of_fifo_packet = sizeof(fifo_packet_with_mag);

	} else {
		_size_of_fifo_packet = sizeof(fifo_packet);
	}

	_max_buf_len = (MPU_MAX_LEN_FIFO_IN_BYTES / _size_of_fifo_packet) * _size_of_fifo_packet;

	return 0;
}

int MPU9250::mpu9250_deinit()
{
	// Leave the IMU in a reset state (turned off).
	int result = _writeReg(MPUREG_PWR_MGMT_1, BIT_H_RESET);

	if (result != 0) {
		DF_LOG_ERR("reset failed");
	}

	// Deallocate the resources for the mag driver, if enabled.
	if (_mag_enabled && _mag != nullptr) {
		delete _mag;
		_mag = nullptr;
	}

	return 0;
}

int MPU9250::pinThread(int cpu)
{
	int ret;
	cpu_set_t cpuset;
	/* Set this cpu-affinity */
	CPU_ZERO(&cpuset);
	CPU_SET(cpu, &cpuset);
	ret = pthread_setaffinity_np(m_thread_id, sizeof(cpu_set_t), &cpuset);
	if (ret != 0) {
		DF_LOG_ERR("pthread_setaffinity_np(%d) failed, ret=%d\n", cpu, ret);
	}

	return ret;
}

int MPU9250::start()
{
	pthread_attr_t attr;
	struct sched_param param = {};
	int result;

	if (_started) {
		DF_LOG_ERR("already started mpu9250!!\n");
		return 0;
	}

	_started = true;

	/* Open the device path specified in the class initialization. */
	// attempt to open device in start()
	result = SPIDevObj::start();

	if (result != 0) {
		DF_LOG_ERR("DevObj start failed");
		DF_LOG_ERR("Unable to open the device path: %s", m_dev_path);
		return result;
	}

	/* Set the bus frequency for register get/set. */
	result = _setBusFrequency(SPI_FREQUENCY_1MHZ);

	if (result != 0) {
		DF_LOG_ERR("failed setting SPI bus frequency: %d", result);
	}

	/* Try to talk to the sensor. */
	uint8_t sensor_id;
	result = _readReg(MPUREG_WHOAMI, sensor_id);

	if (result != 0) {
		DF_LOG_ERR("Unable to communicate with the MPU9250 sensor");
		goto exit;
	}

	if (MPU_WHOAMI_9250 != sensor_id && MPU_WHOAMI_9250_REAL != sensor_id) {
		DF_LOG_ERR("MPU9250 sensor WHOAMI wrong: 0x%X, should be: 0x%X",
			   sensor_id, MPU_WHOAMI_9250);
		result = -1;
		goto exit;
	}

	result = mpu9250_init();

	if (result != 0) {
		DF_LOG_ERR("error: IMU sensor initialization failed, sensor read thread not started");
		goto exit;
	}

	result = DevObj::start();

	if (result != 0) {
		/* This will report failure since the sampling time is NULL */
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

	result = pthread_create(&m_thread_id, &attr, MPU9250::threadFunc, this);

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

int MPU9250::stop()
{
	void *retval;
	int result = mpu9250_deinit();

	if (result != 0) {
		DF_LOG_ERR(
			"error: IMU sensor de-initialization failed.");
		return result;
	}

	result = DevObj::stop();

	if (result != 0) {
		DF_LOG_ERR("DevObj stop failed");
		return result;
	}

	result = pthread_kill(m_thread_id, SIGUSR1);
	if (result != 0) {
		DF_LOG_ERR("pthread_kill: %d", result);
		return result;
	}

	result = pthread_join(m_thread_id, &retval);
	if (result != 0) {
		DF_LOG_ERR("pthread_join: %d", result);
	}

	return 0;
}

int MPU9250::get_fifo_count()
{
	int ret;

	uint8_t write_buf[3];
	uint8_t read_buf[3];
	int16_t *num_bytes = (int16_t*)&read_buf[1];

	struct spi_ioc_transfer spi_transfer;

	write_buf[0] = MPUREG_FIFO_COUNTH | 0x80;

	memset(&spi_transfer, 0, sizeof(spi_ioc_transfer));

	spi_transfer.rx_buf = (unsigned long)read_buf;
	spi_transfer.len = 3;
	spi_transfer.tx_buf = (unsigned long)write_buf;
	spi_transfer.bits_per_word = 8;
	spi_transfer.delay_usecs = 0;

	ret = ::ioctl(m_fd, SPI_IOC_MESSAGE(1), &spi_transfer);

	if (ret == 3) {
		return bswap_16(*num_bytes);

	} else {
		DF_LOG_ERR("FIFO count read failed");
		return ret;
	}
}

void MPU9250::reset_fifo()
{
	int result;

	/*
	 * FIFO Reset is async and may be reset during an incomplete fill.
	 * To avoid this we:
	 * 1. Disable FIFO
	 * 2. Reset FIFO
	 * 3. Enable FIFO
	 */

	result = _writeReg(MPUREG_FIFO_EN, 0);
	if (result != 0) {
		DF_LOG_ERR("MPUREG_FIFO_EN: disable failed");
	}

	result = _modifyReg(MPUREG_USER_CTRL,
			    0,
			    BITS_USER_CTRL_FIFO_RST |
			    BITS_USER_CTRL_FIFO_EN);

	if (result != 0) {
		DF_LOG_ERR("MPUREG_USER_CTRL: fifo rst failed");
	}

	if (_mag_enabled) {
		result = _writeReg(MPUREG_FIFO_EN,
				   BITS_FIFO_ENABLE_TEMP_OUT | BITS_FIFO_ENABLE_GYRO_XOUT
				   | BITS_FIFO_ENABLE_GYRO_YOUT
				   | BITS_FIFO_ENABLE_GYRO_ZOUT | BITS_FIFO_ENABLE_ACCEL
				   | BITS_FIFO_ENABLE_SLV0); // SLV0 is configured for bulk transfer of mag data over I2C

	} else {
		result = _writeReg(MPUREG_FIFO_EN,
				   BITS_FIFO_ENABLE_TEMP_OUT | BITS_FIFO_ENABLE_GYRO_XOUT
				   | BITS_FIFO_ENABLE_GYRO_YOUT
				   | BITS_FIFO_ENABLE_GYRO_ZOUT | BITS_FIFO_ENABLE_ACCEL);
	}

	if (result != 0) {
		DF_LOG_ERR("MPUREG_FIFO_EN: enable failed");
	}
}

void MPU9250::clear_int_status()
{
	int result;
	uint8_t int_status = 0;

	result = _readReg(MPUREG_INT_STATUS, int_status);

	if (result != 0) {
		DF_LOG_ERR("Interrupt status clear failed");
	}
}
void MPU9250::_measure()
{
}

int MPU9250::read_fifo(uint8_t *dst, int num_bytes)
{
	int ret;

	uint8_t write_buf[MPU9250_SW_FIFO_SIZE];

	struct spi_ioc_transfer spi_transfer;

	write_buf[0] = MPUREG_FIFO_R_W | 0x80;

	memset(&spi_transfer, 0, sizeof(spi_ioc_transfer));

	spi_transfer.rx_buf = (unsigned long)dst;
	spi_transfer.len = num_bytes + 1;
	spi_transfer.speed_hz = MPU9250_SPI_FIFO_FREQ;
	spi_transfer.tx_buf = (unsigned long)write_buf;
	spi_transfer.bits_per_word = 8;
	spi_transfer.delay_usecs = 0;

	ret = ::ioctl(m_fd, SPI_IOC_MESSAGE(1), &spi_transfer);
	if (ret != num_bytes + 1) {
		DF_LOG_ERR("ret: %d != %d", ret, num_bytes + 1);
	}

	return 0;
}

int MPU9250::measure()
{
	int sleepTimes = 0;
	int result;
	int bytes_to_read;
	int read_len;
	int packets;
	uint8_t int_status;
	uint8_t fifo_read_buf[8 * MPU9250_MAX_PACKET_SIZE + 1];

	result = _setBusFrequency(SPI_FREQUENCY_1MHZ);
	if (result != 0) {
		DF_LOG_ERR("_setBusFrequency failed: %d\n", result);
	}

	// Get FIFO byte count to read and floor it to the report size.
	do {
		bytes_to_read = get_fifo_count();
		if (bytes_to_read >= _size_of_fifo_packet) {
			break;
		} else if (bytes_to_read < 0) {
			_counters.errors++;
			DF_LOG_ERR("bytes_to_read: %d\n", bytes_to_read);
			goto out;
		} else if (bytes_to_read < _size_of_fifo_packet) {
			/* TODO: replace usleep with nanosleep, check for interrupts, limit the sleep time */
			sleepTimes++;
			usleep(1);
		}
	} while (true);

	if (bytes_to_read > (sizeof(fifo_read_buf) - 1)) {
		reset_fifo();

		_counters.fifo_overflows++;
		DF_LOG_ERR("bytes_to_read too big");

		goto out;
	}

	result = _readReg(MPUREG_INT_STATUS, int_status);

	if (result != 0) {
		_counters.errors++;
		DF_LOG_ERR("MPUREG_INT_STATUS fail: %d\n", result);
		goto out;
	}

	if (int_status & BITS_INT_STATUS_FIFO_OVERFLOW) {
		reset_fifo();

		_counters.fifo_overflows++;
		DF_LOG_ERR("FIFO overflow");

		goto out;
	}

	/* read multiple of packet size */
	bytes_to_read = (bytes_to_read / _size_of_fifo_packet) * _size_of_fifo_packet;

	_counters.fifo_avg_packets = (0.95f * _counters.fifo_avg_packets) + (0.05f * (bytes_to_read / _size_of_fifo_packet));

	read_len = MIN(bytes_to_read, _max_buf_len);

	result = _setBusFrequency(MPU9250_SPI_FIFO_FREQ);

	if (result != 0) {
		DF_LOG_ERR("_setBusFrequency failed: %d\n", result);
	}

	result = read_fifo(fifo_read_buf, read_len);

	if (result != 0) {
		_counters.errors++;
		return 0;
	}

	packets = read_len / _size_of_fifo_packet;
#if MPU9250_DEBUG_TIMING
	if (packets != 1) {
		DF_LOG_INFO("packets: %d", packets);
	}
#endif

	for (unsigned packet_index = 0; packet_index < packets; ++packet_index) {

		fifo_packet_with_mag *report = (fifo_packet_with_mag *)(&fifo_read_buf[1 + packet_index * _size_of_fifo_packet]);

		report->temp = bswap_16(report->temp);

		const float temp_c = (float)report->temp / 361.0f + 35.0f;

		bool publish_accel = true;
		bool publish_gyro = true;
		bool publish_mag = false;

		struct accel_data accel_data;
		struct gyro_data gyro_data;
		struct mag_data mag_data;

		// Use the temperature field to try to detect if we (ever) fall out of sync with
		// the FIFO buffer. If the temperature changes insane amounts, reset the FIFO logic
		// and return early.
		if (!_temp_initialized) {
			// Assume that the temperature should be in a sane range of -40 to 85 deg C which is
			// the specified temperature range, at least to initialize.
			if (temp_c > -40.0f && temp_c < 85.0f) {

				// Initialize the temperature logic.
				_last_temp_c = temp_c;
				DF_LOG_INFO("IMU temperature initialized to: %f", (double) temp_c);
				_temp_initialized = true;
			}

		} else {
			// Once initialized, check for a temperature change of more than 2 degrees which
			// points to a FIFO corruption.
			if (fabsf(temp_c - _last_temp_c) > 2.0f) {
				DF_LOG_ERR(
					"FIFO corrupt, temp difference: %f, last temp: %f, current temp: %f",
					(double)fabsf(temp_c - _last_temp_c), (double)_last_temp_c, (double)temp_c);
				result = _setBusFrequency(SPI_FREQUENCY_1MHZ);
				if (result != 0) {
					DF_LOG_ERR("_setBusFrequency failed: %d\n", result);
				}

				reset_fifo();
				_temp_initialized = false;
				_counters.fifo_corruptions++;

				goto out;
			}

			_last_temp_c = temp_c;
		}

#if MPU9250_CHECK_DUPLICATES
		if (check_duplicate_accel((const uint8_t *)&report->accel_x)) {
			_counters.accel_duplicates++;
		}
#endif

		/* TODO: add ifdef for endianness */
		report->accel_x = bswap_16(report->accel_x);
		report->accel_y = bswap_16(report->accel_y);
		report->accel_z = bswap_16(report->accel_z);

		// Check if the full accel range of the accel has been used. If this occurs, it is
		// either a spike due to a crash/landing or a sign that the vibrations levels
		// measured are excessive.
		if (report->accel_x == INT16_MIN || report->accel_x == INT16_MAX ||
				report->accel_y == INT16_MIN || report->accel_y == INT16_MAX ||
				report->accel_z == INT16_MIN || report->accel_z == INT16_MAX) {
			_counters.accel_range_hits++;
		}

		accel_data.accel_m_s2_x = float(report->accel_x)
								  * (MPU9250_ONE_G / 2048.0f);
		accel_data.accel_m_s2_y = float(report->accel_y)
								  * (MPU9250_ONE_G / 2048.0f);
		accel_data.accel_m_s2_z = float(report->accel_z)
								  * (MPU9250_ONE_G / 2048.0f);

#if MPU9250_CHECK_DUPLICATES
		if (check_duplicate_gyro((const uint8_t *)&report->gyro_x)) {
			_counters.gyro_duplicates++;
		}
#endif

		/* TODO: add ifdef for endianness */
		report->gyro_x = bswap_16(report->gyro_x);
		report->gyro_y = bswap_16(report->gyro_y);
		report->gyro_z = bswap_16(report->gyro_z);

		// Also check the full gyro range, however, this is very unlikely to happen.
		if (report->gyro_x == INT16_MIN || report->gyro_x == INT16_MAX ||
				report->gyro_y == INT16_MIN || report->gyro_y == INT16_MAX ||
				report->gyro_z == INT16_MIN || report->gyro_z == INT16_MAX) {
			_counters.gyro_range_hits++;
		}

		gyro_data.gyro_rad_s_x = float(report->gyro_x) * GYRO_RAW_TO_RAD_S;
		gyro_data.gyro_rad_s_y = float(report->gyro_y) * GYRO_RAW_TO_RAD_S;
		gyro_data.gyro_rad_s_z = float(report->gyro_z) * GYRO_RAW_TO_RAD_S;

		if (_mag_enabled) {
			int mag_error;

			_mag_reads++;
#if MPU9250_CHECK_DUPLICATES
			if (check_duplicate_mag((const uint8_t *)&report->mag_x)) {
				_counters.mag_duplicates++;
			}
#endif

			mag_error = _mag->process((const struct mag_data_packet &)report->mag_st1,
					mag_data.mag_ga_x,
					mag_data.mag_ga_y,
					mag_data.mag_ga_z);

			if (mag_error == MAG_ERROR_DATA_OVERFLOW) {
				_counters.mag_overflows++;

			} else if (mag_error == MAG_ERROR_DATA_OVERRUN) {
				_counters.mag_overruns++;
			}

			if (_mag_reads / 10 >= 1 && mag_error == 0) {
				publish_mag = true;
				_mag_reads = 0;
			}
		}

		_counters.fifo_reads++;

		if (publish_accel) {
			_publish(accel_data);
		}

		if (publish_gyro) {
			_publish(gyro_data);
		}

		if (publish_mag) {
			_publish(mag_data);
		}
	}

out:
	return sleepTimes;
}

#if MPU9250_CHECK_DUPLICATES
bool MPU9250::check_duplicate_accel(const uint8_t *data)
{
	if (memcmp(data, &_last_accel_data, sizeof(_last_accel_data)) == 0) {
		return true;
	}

	memcpy(_last_accel_data, data, sizeof(_last_accel_data));

	return false;
}

bool MPU9250::check_duplicate_gyro(const uint8_t *data)
{
	if (memcmp(data, &_last_gyro_data, sizeof(_last_gyro_data)) == 0) {
		return true;
	}

	memcpy(_last_gyro_data, data, sizeof(_last_gyro_data));

	return false;
}

bool MPU9250::check_duplicate_mag(const uint8_t *data)
{
	if (memcmp(data, &_last_mag_data, sizeof(_last_mag_data)) == 0) {
		return true;
	}

	memcpy(_last_mag_data, data, sizeof(_last_mag_data));

	return false;
}
#endif

#if MPU9250_DEBUG_TIMING
static long int timespec_diff(struct timespec *start, struct timespec *stop)
{
	long int diff = (stop->tv_sec - start->tv_sec) * 1000000000LL + (stop->tv_nsec - start->tv_nsec);
	return diff;
}
#endif

static void timespec_inc(struct timespec *timespec, long dt)
{
	timespec->tv_nsec += dt;

	while (timespec->tv_nsec >= 1000000000) {
		/* timespec nsec overflow */
		timespec->tv_sec++;
		timespec->tv_nsec -= 1000000000;
	}
}

#include <asm/unistd.h>

void* MPU9250::threadFunc(void *arg)
{
	MPU9250 *instance = static_cast<MPU9250*>(arg);

	long wakeup_period_ns = MPU9250_MEASURE_INTERVAL_US * 1000 - (7 * 1000);
	struct timespec next_wakeup;
#if MPU9250_DEBUG_TIMING
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

	DF_LOG_ERR("MPU9250 TID: %d", (long int)syscall(__NR_gettid));

	clock_gettime(CLOCK_MONOTONIC, &next_wakeup);
	while (1) {
#if MPU9250_DEBUG_TIMING
		clock_gettime(CLOCK_MONOTONIC, &start_time);
#endif
		int jitter = instance->measure();
#if MPU9250_DEBUG_TIMING
		clock_gettime(CLOCK_MONOTONIC, &end_time);
#endif

#if MPU9250_DEBUG_TIMING
		sleep_dt = timespec_diff(&sleep_time, &start_time);
		exec_dt = timespec_diff(&start_time, &end_time);
		sleep_dt_mean = (sleep_dt_mean + sleep_dt) / 2.0;
		exec_dt_mean = (exec_dt_mean + exec_dt) / 2.0;

		if (jitter != 0) {
			if (false)
				printf("MPU9250 jitter: %d\n", jitter);
		}


		if (stats_print_cnt / 100 != 0) {
			printf("MPU9250 sleep_dt_mean: %f\n", sleep_dt_mean);
			printf("MPU9250 exec_dt_mean: %f\n", exec_dt_mean);
			stats_print_cnt = 0;
		} else {
			stats_print_cnt++;
		}
#endif

		timespec_inc(&next_wakeup, wakeup_period_ns + jitter);

#if MPU9250_DEBUG_TIMING
		clock_gettime(CLOCK_MONOTONIC, &sleep_time);
#endif
		int ret = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_wakeup, NULL);
		if (ret == EINTR) {
			DF_LOG_ERR("MPU9250 someone sent me a signal");
			return NULL;
		} else if (ret != 0) {
			DF_LOG_ERR("MPU9250 fail to sleep: %d", ret);
			return NULL;
		}
	}

	return NULL;
}
