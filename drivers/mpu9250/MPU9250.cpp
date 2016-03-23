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
#include "math.h"
#include "DriverFramework.hpp"
#include "MPU9250.hpp"


#define MPUREG_WHOAMI			0x75
#define MPUREG_SMPLRT_DIV		0x19
#define MPUREG_CONFIG			0x1A
#define MPUREG_GYRO_CONFIG		0x1B
#define MPUREG_ACCEL_CONFIG		0x1C
#define MPUREG_ACCEL_CONFIG2		0x1D
#define MPUREG_LPACCEL_ODR		0x1E
#define MPUREG_WOM_THRESH		0x1F
#define MPUREG_FIFO_EN			0x23
#define MPUREG_I2C_MST_CTRL		0x24
#define MPUREG_I2C_SLV0_ADDR		0x25
#define MPUREG_I2C_SLV0_REG		0x26
#define MPUREG_I2C_SLV0_CTRL		0x27
#define MPUREG_I2C_SLV1_ADDR		0x28
#define MPUREG_I2C_SLV1_REG		0x29
#define MPUREG_I2C_SLV1_CTRL		0x2A
#define MPUREG_I2C_SLV2_ADDR		0x2B
#define MPUREG_I2C_SLV2_REG		0x2C
#define MPUREG_I2C_SLV2_CTRL		0x2D
#define MPUREG_I2C_SLV3_ADDR		0x2E
#define MPUREG_I2C_SLV3_REG		0x2F
#define MPUREG_I2C_SLV3_CTRL		0x30
#define MPUREG_I2C_SLV4_ADDR		0x31
#define MPUREG_I2C_SLV4_REG		0x32
#define MPUREG_I2C_SLV4_DO		0x33
#define MPUREG_I2C_SLV4_CTRL		0x34
#define MPUREG_I2C_SLV4_DI		0x35
#define MPUREG_I2C_MST_STATUS		0x36
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
#define MPUREG_EXT_SENS_DATA_00		0x49
#define MPUREG_I2C_SLV0_D0		0x63
#define MPUREG_I2C_SLV1_D0		0x64
#define MPUREG_I2C_SLV2_D0		0x65
#define MPUREG_I2C_SLV3_D0		0x66
#define MPUREG_I2C_MST_DELAY_CTRL	0x67
#define MPUREG_SIGNAL_PATH_RESET	0x68
#define MPUREG_MOT_DETECT_CTRL		0x69
#define MPUREG_USER_CTRL		0x6A
#define MPUREG_PWR_MGMT_1		0x6B
#define MPUREG_PWR_MGMT_2		0x6C
#define MPUREG_FIFO_COUNTH		0x72
#define MPUREG_FIFO_COUNTL		0x73
#define MPUREG_FIFO_R_W			0x74

// Configuration bits MPU 9250
#define BIT_SLEEP			0x40
#define BIT_H_RESET			0x80
#define MPU_CLK_SEL_AUTO		0x01

#define BITS_GYRO_ST_X			0x80
#define BITS_GYRO_ST_Y			0x40
#define BITS_GYRO_ST_Z			0x20
#define BITS_FS_250DPS			0x00
#define BITS_FS_500DPS			0x08
#define BITS_FS_1000DPS			0x10
#define BITS_FS_2000DPS			0x18
#define BITS_FS_MASK			0x18

#define BITS_DLPF_CFG_250HZ		0x00
#define BITS_DLPF_CFG_184HZ		0x01
#define BITS_DLPF_CFG_92HZ		0x02
#define BITS_DLPF_CFG_41HZ		0x03
#define BITS_DLPF_CFG_20HZ		0x04
#define BITS_DLPF_CFG_10HZ		0x05
#define BITS_DLPF_CFG_5HZ		0x06
#define BITS_DLPF_CFG_3600HZ		0x07
#define BITS_DLPF_CFG_MASK		0x07

#define BITS_FIFO_ENABLE_TEMP_OUT	0x80
#define BITS_FIFO_ENABLE_GYRO_XOUT	0x40
#define BITS_FIFO_ENABLE_GYRO_YOUT	0x20
#define BITS_FIFO_ENABLE_GYRO_ZOUT	0x10
#define BITS_FIFO_ENABLE_ACCEL		0x08

#define BITS_ACCEL_CONFIG_16G		0x18

#define BIT_RAW_RDY_EN			0x01
#define BIT_INT_ANYRD_2CLEAR		0x10

#define MPU9250_ONE_G					9.80665f



using namespace DriverFramework;

int MPU9250::mpu9250_init()
{
	/* Zero the struct */
	m_synchronize.lock();
	m_sensor_data.accel_m_s2_x = 0.0f;
	m_sensor_data.accel_m_s2_y = 0.0f;
	m_sensor_data.accel_m_s2_z = 0.0f;
	m_sensor_data.temp_c = 0.0f;
	m_sensor_data.gyro_rad_s_x = 0.0f;
	m_sensor_data.gyro_rad_s_y = 0.0f;
	m_sensor_data.gyro_rad_s_z = 0.0f;

	m_sensor_data.last_read_time_usec = 0;
	m_sensor_data.read_counter = 0;
	m_synchronize.unlock();


	int result = _writeReg(MPUREG_PWR_MGMT_1, BIT_H_RESET);

	if (result != 0) {
		DF_LOG_ERR("reset failed");
	}

	usleep(1000);

	result = _writeReg(MPUREG_PWR_MGMT_1, MPU_CLK_SEL_AUTO);

	if (result != 0) {
		DF_LOG_ERR("clock selection failed");
	}

	usleep(1000);

	result = _writeReg(MPUREG_PWR_MGMT_2, 0);

	if (result != 0) {
		DF_LOG_ERR("clock selection failed");
	}

	usleep(1000);

	result = _writeReg(MPUREG_FIFO_EN, BITS_FIFO_ENABLE_TEMP_OUT |
			   BITS_FIFO_ENABLE_GYRO_XOUT |
			   BITS_FIFO_ENABLE_GYRO_YOUT |
			   BITS_FIFO_ENABLE_GYRO_ZOUT);

	if (result != 0) {
		DF_LOG_ERR("FIFO enable failed");
	}

	usleep(1000);

	/*
	 * A samplerate_divider of 0 should give 1000Hz:
	 *
	 * sample_rate = internal_sample_rate / (1+samplerate_divider)
	 */
	uint8_t samplerate_divider = 0;
	result = _writeReg(MPUREG_FIFO_EN, samplerate_divider);

	if (result != 0) {
		DF_LOG_ERR("sample rate config failed");
	}

	usleep(1000);

	result = _writeReg(MPUREG_CONFIG, BITS_DLPF_CFG_41HZ);

	if (result != 0) {
		DF_LOG_ERR("DLPF config failed");
	}

	usleep(1000);

	result = _writeReg(MPUREG_GYRO_CONFIG, BITS_FS_2000DPS);

	if (result != 0) {
		DF_LOG_ERR("Gyro scale config failed");
	}

	usleep(1000);

	result = _writeReg(MPUREG_ACCEL_CONFIG, BITS_ACCEL_CONFIG_16G);

	if (result != 0) {
		DF_LOG_ERR("Accel scale config failed");
	}

	usleep(1000);


	return 0;
}

int MPU9250::start()
{

	/* Open the device path specified in the class initialization. */
	// attempt to open device in start()
	int result = SPIDevObj::start();

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

	if (sensor_id != MPU_WHOAMI_9250) {
		DF_LOG_ERR("MPU9250 sensor WHOAMI wrong: 0x%X, should be: 0x%X", sensor_id, MPU_WHOAMI_9250);
		result = -1;
		goto exit;
	}

	result = mpu9250_init();

	/* Set the bus frequency for normal operation. */
	result = _setBusFrequency(SPI_FREQUENCY_20MHZ);

	if (result != 0) {
		DF_LOG_ERR("failed setting SPI bus frequency: %d", result);
	}

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

	if (result != 0) {
		devClose();
	}

	return result;
}

int MPU9250::stop()
{
	int result = DevObj::stop();

	if (result != 0) {
		DF_LOG_ERR("DevObj stop failed");
		return result;
	}

	result = devClose();

	if (result != 0) {
		DF_LOG_ERR("device close failed");
		return result;
	}

	return 0;
}

void MPU9250::_measure()
{
#pragma pack(push, 1)
	struct int_status_report {
		int16_t		accel_x;
		int16_t		accel_y;
		int16_t		accel_z;
		int16_t		temp;
		int16_t		gyro_x;
		int16_t		gyro_y;
		int16_t		gyro_z;
	} report;
#pragma pack(pop)

	_bulkRead(MPUREG_ACCEL_XOUT_H, (uint8_t *)&report, sizeof(report));

	/* TODO: add ifdef for endianness */
	report.accel_x = swap16(report.accel_x);
	report.accel_y = swap16(report.accel_y);
	report.accel_z = swap16(report.accel_z);
	report.temp = swap16(report.temp);
	report.gyro_x = swap16(report.gyro_x);
	report.gyro_y = swap16(report.gyro_y);
	report.gyro_z = swap16(report.gyro_z);

	m_synchronize.lock();

	m_sensor_data.accel_m_s2_x = float(report.accel_x) * (MPU9250_ONE_G / 2048.0f);
	m_sensor_data.accel_m_s2_y = float(report.accel_y) * (MPU9250_ONE_G / 2048.0f);
	m_sensor_data.accel_m_s2_z = float(report.accel_z) * (MPU9250_ONE_G / 2048.0f);
	m_sensor_data.temp_c = float(report.temp) / 361.0f + 35.0f;
	m_sensor_data.gyro_rad_s_x = float(report.gyro_x) * GYRO_RAW_TO_RAD_S;
	m_sensor_data.gyro_rad_s_y = float(report.gyro_y) * GYRO_RAW_TO_RAD_S;
	m_sensor_data.gyro_rad_s_z = float(report.gyro_z) * GYRO_RAW_TO_RAD_S;

	m_sensor_data.last_read_time_usec = DriverFramework::offsetTime();
	m_sensor_data.read_counter++;

	_publish(m_sensor_data);

	m_synchronize.signal();
	m_synchronize.unlock();
}

int MPU9250::_publish(struct imu_sensor_data &data)
{
	// TBD
	return -1;
}
