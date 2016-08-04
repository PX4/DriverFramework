/****************************************************************************
 *
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

#include <stdint.h>
#include <string.h>
#include "math.h"
#include "DriverFramework.hpp"
#include "MPU6050.hpp"

struct fifo_packet {
	int16_t accel_x;
	int16_t accel_y;
	int16_t accel_z;
	int16_t temp;
	int16_t gyro_x;
	int16_t gyro_y;
	int16_t gyro_z;
};

#define MPU6050_ONE_G	9.80665f

#define MIN(_x, _y) (_x) > (_y) ? (_y) : (_x)

#ifndef M_PI_F
#define M_PI_F 3.14159265358979323846f
#endif

// LSB sensitivity from the datasheet is 16.4 LSB/degree/s at +-2000 degrees/s
// and deg to rad conversion
#define GYRO_RAW_TO_RAD_S 	 (M_PI_F / 180.0f / 16.4f)

#define DIR_READ			0x80
#define DIR_WRITE			0x00

// MPU 6050 registers
#define MPUREG_WHOAMI			0x75
#define MPUREG_SMPLRT_DIV		0x19
#define MPUREG_CONFIG			0x1A
#define MPUREG_GYRO_CONFIG		0x1B
#define MPUREG_ACCEL_CONFIG		0x1C
#define MPUREG_FIFO_EN			0x23
#define MPUREG_INT_PIN_CFG		0x37
#define MPUREG_INT_ENABLE		0x38
#define MPUREG_INT_STATUS		0x3A
#define MPUREG_ACCEL_XOUT_H		0x3B
#define MPUREG_ACCEL_XOUT_L		0x3C
#define MPUREG_ACCEL_YOUT_H		0x3D
#define MPUREG_ACCEL_YOUT_L		0x3E
#define MPUREG_ACCEL_ZOUT_H		0x3F
#define MPUREG_ACCEL_ZOUT_L		0x40
#define MPUREG_TEMP_OUT_H		0x41
#define MPUREG_TEMP_OUT_L		0x42
#define MPUREG_GYRO_XOUT_H		0x43
#define MPUREG_GYRO_XOUT_L		0x44
#define MPUREG_GYRO_YOUT_H		0x45
#define MPUREG_GYRO_YOUT_L		0x46
#define MPUREG_GYRO_ZOUT_H		0x47
#define MPUREG_GYRO_ZOUT_L		0x48
#define MPUREG_USER_CTRL		0x6A
#define MPUREG_PWR_MGMT_1		0x6B
#define MPUREG_PWR_MGMT_2		0x6C
#define MPUREG_FIFO_COUNTH		0x72
#define MPUREG_FIFO_COUNTL		0x73
#define MPUREG_FIFO_R_W			0x74
#define MPUREG_PRODUCT_ID		0x0C
#define MPUREG_TRIM1			0x0D
#define MPUREG_TRIM2			0x0E
#define MPUREG_TRIM3			0x0F
#define MPUREG_TRIM4			0x10

// Configuration bits MPU 6050
#define BIT_SLEEP			0x40
#define BIT_H_RESET			0x80
#define BITS_CLKSEL			0x07
#define MPU_CLK_SEL_PLLGYROX		0x01
#define MPU_CLK_SEL_PLLGYROZ		0x03
#define MPU_EXT_SYNC_GYROX		0x02
#define BITS_GYRO_ST_X			0x80
#define BITS_GYRO_ST_Y			0x40
#define BITS_GYRO_ST_Z			0x20
#define BITS_FS_250DPS			0x00
#define BITS_FS_500DPS			0x08
#define BITS_FS_1000DPS			0x10
#define BITS_FS_2000DPS			0x18
#define BITS_ACCEL_CONFIG_2G 0x00
#define BITS_ACCEL_CONFIG_4G 0x08
#define BITS_ACCEL_CONFIG_8G 0x10
#define BITS_ACCEL_CONFIG_16G 0x18
#define BIT_INT_ANYRD_2CLEAR		0x10
#define BIT_RAW_RDY_EN			0x01
#define BIT_I2C_IF_DIS			0x10
#define BIT_INT_STATUS_DATA		0x01
#define BITS_USER_CTRL_FIFO_RST 0x04
#define BITS_USER_CTRL_I2C_MST_EN 0x20
#define BITS_USER_CTRL_FIFO_EN 0x40
#define BITS_FIFO_ENABLE_TEMP_OUT 0x80
#define BITS_FIFO_ENABLE_GYRO_XOUT 0x40
#define BITS_FIFO_ENABLE_GYRO_YOUT 0x20
#define BITS_FIFO_ENABLE_GYRO_ZOUT 0x10
#define BITS_FIFO_ENABLE_ACCEL 0x08
#define BITS_INT_STATUS_FIFO_OVERFLOW 0x10
#define BITS_DLPF_CFG_260HZ		0x00
#define BITS_DLPF_CFG_184HZ		0x01
#define BITS_DLPF_CFG_94HZ		0x02
#define BITS_DLPF_CFG_44HZ		0x03
#define BITS_DLPF_CFG_21HZ		0x04
#define BITS_DLPF_CFG_10HZ		0x05
#define BITS_DLPF_CFG_5HZ		0x06

// Length of the FIFO used by the sensor to buffer unread
// sensor data. The FIFO size of the MPU6050 is 1024 bytes.
// However, we sample and read with 1 kHz, so that most often
// there is just one packet in the FIFO.
// Including some slack, we allocate memory for four packets.
#define MPU_MAX_LEN_FIFO_IN_BYTES (4 * sizeof(fifo_packet))

// Uncomment to allow additional debug output to be generated.
// #define MPU6050_DEBUG 1

using namespace DriverFramework;

int MPU6050::mpu6050_init()
{
	/* Zero the struct */
	m_synchronize.lock();

	m_sensor_data.accel_m_s2_x = 0.0f;
	m_sensor_data.accel_m_s2_y = 0.0f;
	m_sensor_data.accel_m_s2_z = 0.0f;
	m_sensor_data.gyro_rad_s_x = 0.0f;
	m_sensor_data.gyro_rad_s_y = 0.0f;
	m_sensor_data.gyro_rad_s_z = 0.0f;
	m_sensor_data.mag_ga_x = 0.0f;
	m_sensor_data.mag_ga_y = 0.0f;
	m_sensor_data.mag_ga_z = 0.0f;
	m_sensor_data.temp_c = 0.0f;

	m_sensor_data.read_counter = 0;
	m_sensor_data.error_counter = 0;
	m_sensor_data.fifo_overflow_counter = 0;
	m_sensor_data.fifo_corruption_counter = 0;
	m_sensor_data.gyro_range_hit_counter = 0;
	m_sensor_data.accel_range_hit_counter = 0;

	m_sensor_data.fifo_sample_interval_us = 0;
	m_sensor_data.is_last_fifo_sample = false;

	m_synchronize.unlock();

	int result = _setSlaveConfig(MPU6050_SLAVE_ADDRESS,
				     MPU6050_BUS_FREQUENCY_IN_KHZ,
				     MPU6050_TRANSFER_TIMEOUT_IN_USECS);

	if (result < 0) {
		DF_LOG_ERR("Could not set slave config");
	}

	uint8_t bits = BIT_H_RESET;
	result = _writeReg(MPUREG_PWR_MGMT_1, &bits, 1);

	if (result < 0) {
		DF_LOG_ERR("Reset failed");
		return -1;
	}

	usleep(100000);

	bits = MPU_CLK_SEL_PLLGYROZ;
	result = _writeReg(MPUREG_PWR_MGMT_1, &bits, 1);

	if (result < 0) {
		DF_LOG_ERR("Wakeup sensor failed");
		return -1;
	}

	usleep(1000);

	// Don't set sensors into standby mode
	bits = 0;
	result = _writeReg(MPUREG_PWR_MGMT_2, &bits, 1);

	if (result < 0) {
		DF_LOG_ERR("enable failed");
		return -1;
	}

	usleep(1000);

	// Enable FIFO.
	bits = BITS_USER_CTRL_FIFO_RST | BITS_USER_CTRL_FIFO_EN;
	result = _writeReg(MPUREG_USER_CTRL, &bits, 1);

	if (result < 0) {
		DF_LOG_ERR("User ctrl failed");
		return -1;
	}

	usleep(1000);

	bits = BITS_FIFO_ENABLE_TEMP_OUT | BITS_FIFO_ENABLE_GYRO_XOUT
	       | BITS_FIFO_ENABLE_GYRO_YOUT
	       | BITS_FIFO_ENABLE_GYRO_ZOUT | BITS_FIFO_ENABLE_ACCEL;
	result = _writeReg(MPUREG_FIFO_EN,
			   &bits, 1);

	if (result < 0) {
		DF_LOG_ERR("FIFO enable failed");
		return -1;
	}

	usleep(1000);

	// Set sample frequency
	bits = BITS_DLPF_CFG_260HZ;
	result = _writeReg(MPUREG_CONFIG, &bits, 1);

	if (result < 0) {
		DF_LOG_ERR("Config failed");
		return -1;
	}

	usleep(1000);

	// Set the sample rate divider
	// sample rate = Gyroscope_Output_Rate / (1 + sample_divider)
	//
	// If DLPF is disabled (0 or 7) Gyroscope_Output_Rate = 8 kHz
	// otherwise Gyroscope_Output_Rate = 1kHz
	bits = 7;
	result = _writeReg(MPUREG_SMPLRT_DIV, &bits, 1);

	if (result < 0) {
		DF_LOG_ERR("Set sample divider failed");
		return -1;
	}

	usleep(1000);

	// Set the gyro resolution
	bits = BITS_FS_2000DPS;
	result = _writeReg(MPUREG_GYRO_CONFIG, &bits, 1);

	if (result < 0) {
		DF_LOG_ERR("Gyro scale config failed");
		return -1;
	}

	usleep(1000);

	// Set the accel resolution
	bits = BITS_ACCEL_CONFIG_16G;
	result = _writeReg(MPUREG_ACCEL_CONFIG, &bits, 1);

	if (result < 0) {
		DF_LOG_ERR("Accel scale config failed");
		return -1;
	}

	usleep(1000);

	// Enable/clear the FIFO of any residual data
	reset_fifo();

	return 0;
}

int MPU6050::mpu6050_deinit()
{
	// Leave the IMU in a reset state (turned off).
	uint8_t bits = BIT_H_RESET;
	int result = _writeReg(MPUREG_PWR_MGMT_1, &bits, 1);

	if (result != 0) {
		DF_LOG_ERR("reset failed");
	}

	return 0;
}

int MPU6050::start()
{
	/* Open the device path specified in the class initialization. */
	// attempt to open device in start()
	int result = I2CDevObj::start();

	if (result < 0) {
		DF_LOG_ERR("DevObj start failed");
		DF_LOG_ERR("Unable to open the device path: %s", m_dev_path);
		return result;
	}

	result = _setSlaveConfig(MPU6050_SLAVE_ADDRESS,
				 MPU6050_BUS_FREQUENCY_IN_KHZ,
				 MPU6050_TRANSFER_TIMEOUT_IN_USECS);

	if (result < 0) {
		DF_LOG_ERR("Could not set slave config");
	}

	/* Try to talk to the sensor. */
	uint8_t sensor_id;
	result = _readReg(MPUREG_WHOAMI, &sensor_id, 1);

	if (result < 0) {
		DF_LOG_ERR("Unable to communicate with the sensor");
		return -1;
	}

	if (sensor_id != MPU_WHOAMI_6050) {
		DF_LOG_ERR("MPU6050 sensor WHOAMI wrong: 0x%X, should be: 0x%X",
			   sensor_id, MPU_WHOAMI_6050);
		result = -1;
		goto exit;
	}

	result = mpu6050_init();

	if (result != 0) {
		DF_LOG_ERR("error: IMU sensor initialization failed, sensor read thread not started");
		goto exit;
	}

	result = DevObj::start();

	if (result != 0) {
		DF_LOG_ERR("DevObj start failed");
		return result;
	}

exit:
	return result;
}

int MPU6050::stop()
{
	int result = mpu6050_deinit();

	if (result < 0) {
		DF_LOG_ERR(
			"error: IMU sensor de-initialization failed.");
		return result;
	}

	result = DevObj::stop();

	if (result != 0) {
		DF_LOG_ERR("DevObj stop failed");
		return result;
	}

	// We need to wait so that all measure calls are finished before
	// closing the device.
	usleep(10000);

	return 0;
}

int MPU6050::get_fifo_count()
{
	int16_t num_bytes = 0x0;

	int result = _readReg(MPUREG_FIFO_COUNTH, (uint8_t *) &num_bytes, sizeof(num_bytes));

	if (result < 0) {
		DF_LOG_ERR("FIFO count read failed");
		return 0;
	}

	num_bytes = swap16(num_bytes);

	return num_bytes;

}

void MPU6050::reset_fifo()
{
	uint8_t bits = BITS_USER_CTRL_FIFO_RST | BITS_USER_CTRL_FIFO_EN;
	int result = _writeReg(MPUREG_USER_CTRL, &bits, 1);

	if (result < 0) {
		DF_LOG_ERR("FIFO reset failed");
	}
}

void MPU6050::_measure()
{
	uint8_t int_status = 0;
	int result = _readReg(MPUREG_INT_STATUS, &int_status, 1);

	if (result < 0) {
		m_synchronize.lock();
		++m_sensor_data.error_counter;
		m_synchronize.unlock();
		return;
	}

	if (int_status & BITS_INT_STATUS_FIFO_OVERFLOW) {
		reset_fifo();

		m_synchronize.lock();
		++m_sensor_data.fifo_overflow_counter;
		DF_LOG_ERR("FIFO overflow: %d", (int)m_sensor_data.fifo_overflow_counter);
		m_synchronize.unlock();

		return;
	}

	int size_of_fifo_packet = sizeof(fifo_packet);

	// Get FIFO byte count to read and floor it to the report size.
	int bytes_to_read = get_fifo_count() / size_of_fifo_packet * size_of_fifo_packet;
	_packets_per_cycle_filtered = (0.95f * _packets_per_cycle_filtered) + (0.05f * (bytes_to_read / size_of_fifo_packet));

	if (bytes_to_read <= 0) {
		m_synchronize.lock();
		++m_sensor_data.error_counter;
		m_synchronize.unlock();
		return;
	}

	// Allocate a buffer large enough for n complete packets, read from the
	// sensor FIFO.
	const unsigned buf_len = (MPU_MAX_LEN_FIFO_IN_BYTES / size_of_fifo_packet) * size_of_fifo_packet;
	uint8_t fifo_read_buf[buf_len];

	const unsigned read_len = MIN((unsigned)bytes_to_read, buf_len);
	memset(fifo_read_buf, 0x0, buf_len);

	result = _readReg(MPUREG_FIFO_R_W, fifo_read_buf, read_len);

	if (result < 0) {
		m_synchronize.lock();
		++m_sensor_data.error_counter;
		m_synchronize.unlock();
		return;
	}

	for (unsigned packet_index = 0; packet_index < (read_len / size_of_fifo_packet); ++packet_index) {

		fifo_packet *report = (fifo_packet *)(&fifo_read_buf[packet_index	* size_of_fifo_packet]);

		report->accel_x = swap16(report->accel_x);
		report->accel_y = swap16(report->accel_y);
		report->accel_z = swap16(report->accel_z);
		report->temp = swap16(report->temp);
		report->gyro_x = swap16(report->gyro_x);
		report->gyro_y = swap16(report->gyro_y);
		report->gyro_z = swap16(report->gyro_z);


		// Check if the full accel range of the accel has been used. If this occurs, it is
		// either a spike due to a crash/landing or a sign that the vibrations levels
		// measured are excessive.
		if (report->accel_x == INT16_MIN || report->accel_x == INT16_MAX ||
		    report->accel_y == INT16_MIN || report->accel_y == INT16_MAX ||
		    report->accel_z == INT16_MIN || report->accel_z == INT16_MAX) {
			m_synchronize.lock();
			++m_sensor_data.accel_range_hit_counter;
			m_synchronize.unlock();
		}

		// Also check the full gyro range, however, this is very unlikely to happen.
		if (report->gyro_x == INT16_MIN || report->gyro_x == INT16_MAX ||
		    report->gyro_y == INT16_MIN || report->gyro_y == INT16_MAX ||
		    report->gyro_z == INT16_MIN || report->gyro_z == INT16_MAX) {
			m_synchronize.lock();
			++m_sensor_data.gyro_range_hit_counter;
			m_synchronize.unlock();
		}

		const float temp_c = float(report->temp) / 340.0f + 36.53f;

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
					fabs(temp_c - _last_temp_c), (double)_last_temp_c, (double)temp_c);
				reset_fifo();
				_temp_initialized = false;
				m_synchronize.lock();
				++m_sensor_data.fifo_corruption_counter;
				m_synchronize.unlock();
				return;
			}

			_last_temp_c = temp_c;
		}

		m_synchronize.lock();
		m_sensor_data.accel_m_s2_x = float(report->accel_x)
					     * (MPU6050_ONE_G / 2048.0f);
		m_sensor_data.accel_m_s2_y = float(report->accel_y)
					     * (MPU6050_ONE_G / 2048.0f);
		m_sensor_data.accel_m_s2_z = float(report->accel_z)
					     * (MPU6050_ONE_G / 2048.0f);
		m_sensor_data.temp_c = temp_c;
		m_sensor_data.gyro_rad_s_x = float(report->gyro_x) * GYRO_RAW_TO_RAD_S;
		m_sensor_data.gyro_rad_s_y = float(report->gyro_y) * GYRO_RAW_TO_RAD_S;
		m_sensor_data.gyro_rad_s_z = float(report->gyro_z) * GYRO_RAW_TO_RAD_S;

		// Pass on the sampling interval between FIFO samples at 1kHz.
		m_sensor_data.fifo_sample_interval_us = 1000000 / MPU6050_MEASURE_INTERVAL_US
							/ _packets_per_cycle_filtered;

		// Flag if this is the last sample, and _publish() should wrap up the data it has received.
		m_sensor_data.is_last_fifo_sample = ((packet_index + 1) == (read_len / size_of_fifo_packet));

		++m_sensor_data.read_counter;

		// Generate debug output every second, assuming that a sample is generated every
		// 125 usecs
#ifdef MPU6050_DEBUG

		if (++m_sensor_data.read_counter % (1000000 / 125) == 0) {

			DF_LOG_INFO("IMU: accel: [%f, %f, %f]",
				    (double)m_sensor_data.accel_m_s2_x,
				    (double)m_sensor_data.accel_m_s2_y,
				    (double)m_sensor_data.accel_m_s2_z);
			DF_LOG_INFO("     gyro:  [%f, %f, %f]",
				    (double)m_sensor_data.gyro_rad_s_x,
				    (double)m_sensor_data.gyro_rad_s_y,
				    (double)m_sensor_data.gyro_rad_s_z);
			DF_LOG_INFO("    temp:  %f C", (double)m_sensor_data.temp_c);
		}

#endif

		_publish(m_sensor_data);

		m_synchronize.signal();
		m_synchronize.unlock();
	}
}

int MPU6050::_publish(struct imu_sensor_data &data)
{
	// TBD
	return -1;
}
