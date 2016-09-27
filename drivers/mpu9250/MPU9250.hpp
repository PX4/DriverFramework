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

#pragma once

#include "ImuSensor.hpp"
#include "MPU9250_mag.hpp"

namespace DriverFramework
{
#define MPUREG_WHOAMI			0x75
#define MPUREG_SMPLRT_DIV		0x19
#define MPUREG_CONFIG			0x1A
#define MPUREG_GYRO_CONFIG		0x1B
#define MPUREG_ACCEL_CONFIG		0x1C
#define MPUREG_ACCEL_CONFIG2	0x1D
#define MPUREG_LPACCEL_ODR		0x1E
#define MPUREG_WOM_THRESH		0x1F
#define MPUREG_FIFO_EN			0x23
#define MPUREG_I2C_MST_CTRL		0x24
#define MPUREG_I2C_SLV0_ADDR	0x25
#define MPUREG_I2C_SLV0_REG		0x26
#define MPUREG_I2C_SLV0_CTRL	0x27
#define MPUREG_I2C_SLV1_ADDR	0x28
#define MPUREG_I2C_SLV1_REG		0x29
#define MPUREG_I2C_SLV1_CTRL	0x2A
#define MPUREG_I2C_SLV2_ADDR	0x2B
#define MPUREG_I2C_SLV2_REG		0x2C
#define MPUREG_I2C_SLV2_CTRL	0x2D
#define MPUREG_I2C_SLV3_ADDR	0x2E
#define MPUREG_I2C_SLV3_REG		0x2F
#define MPUREG_I2C_SLV3_CTRL	0x30
#define MPUREG_I2C_SLV4_ADDR	0x31
#define MPUREG_I2C_SLV4_REG		0x32
#define MPUREG_I2C_SLV4_DO		0x33
#define MPUREG_I2C_SLV4_CTRL	0x34
#define MPUREG_I2C_SLV4_DI		0x35
#define MPUREG_I2C_MST_STATUS	0x36
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
#define MPUREG_EXT_SENS_DATA_00	0x49
#define MPUREG_I2C_SLV0_D0		0x63
#define MPUREG_I2C_SLV1_D0		0x64
#define MPUREG_I2C_SLV2_D0		0x65
#define MPUREG_I2C_SLV3_D0		0x66
#define MPUREG_I2C_MST_DELAY_CTRL	0x67
#define MPUREG_SIGNAL_PATH_RESET	0x68
#define MPUREG_MOT_DETECT_CTRL	0x69
#define MPUREG_USER_CTRL		0x6A
#define MPUREG_PWR_MGMT_1		0x6B
#define MPUREG_PWR_MGMT_2		0x6C
#define MPUREG_FIFO_COUNTH		0x72
#define MPUREG_FIFO_COUNTL		0x73
#define MPUREG_FIFO_R_W			0x74

// Length of the FIFO used by the sensor to buffer unread
// sensor data.
#define MPU_MAX_LEN_FIFO_IN_BYTES 512

// Configuration bits MPU 9250
#define BIT_SLEEP			0x40
#define BIT_H_RESET			0x80
#define MPU_CLK_SEL_AUTO	0x01

#define BITS_USER_CTRL_FIFO_EN		0x40
#define BITS_USER_CTRL_FIFO_RST		0x04
#define BITS_USER_CTRL_I2C_MST_EN  	0x20
#define BITS_USER_CTRL_I2C_IF_DIS  	0x10
#define BITS_USER_CTRL_I2C_MST_RST  	0x02

#define BITS_CONFIG_FIFO_MODE_OVERWRITE	0x00
#define BITS_CONFIG_FIFO_MODE_STOP	0x40

#define BITS_GYRO_ST_X			0x80
#define BITS_GYRO_ST_Y			0x40
#define BITS_GYRO_ST_Z			0x20
#define BITS_FS_250DPS			0x00
#define BITS_FS_500DPS			0x08
#define BITS_FS_1000DPS			0x10
#define BITS_FS_2000DPS			0x18
#define BITS_FS_MASK			0x18
// This is FCHOICE_B which is the inverse of FCHOICE
#define BITS_BW_3600HZ			0x02
// The FCHOICE bits are the same for all Bandwidths below 3600 Hz.
#define BITS_BW_LT3600HZ		0x00

#define BITS_DLPF_CFG_250HZ		0x00
#define BITS_DLPF_CFG_184HZ		0x01
#define BITS_DLPF_CFG_92HZ		0x02
#define BITS_DLPF_CFG_41HZ		0x03
#define BITS_DLPF_CFG_20HZ		0x04
#define BITS_DLPF_CFG_10HZ		0x05
#define BITS_DLPF_CFG_5HZ		0x06
#define BITS_DLPF_CFG_3600HZ	0x07
#define BITS_DLPF_CFG_MASK		0x07

#define BITS_FIFO_ENABLE_TEMP_OUT	0x80
#define BITS_FIFO_ENABLE_GYRO_XOUT	0x40
#define BITS_FIFO_ENABLE_GYRO_YOUT	0x20
#define BITS_FIFO_ENABLE_GYRO_ZOUT	0x10
#define BITS_FIFO_ENABLE_ACCEL		0x08
#define BITS_FIFO_ENABLE_SLV2		0x04
#define BITS_FIFO_ENABLE_SLV1		0x02
#define BITS_FIFO_ENABLE_SLV0		0x01

#define BITS_ACCEL_CONFIG_16G		0x18

// This is ACCEL_FCHOICE_B which is the inverse of ACCEL_FCHOICE
#define BITS_ACCEL_CONFIG2_BW_1130HZ	0x08

#define BITS_I2C_SLV0_EN    0x80
#define BITS_I2C_SLV0_READ_8BYTES 0x08
#define BITS_I2C_SLV1_EN    0x80
#define BITS_I2C_SLV1_DIS   0x00
#define BITS_I2C_SLV2_EN    0x80
#define BITS_I2C_SLV4_EN    0x80
#define BITS_I2C_SLV4_DONE  0x40

#define BITS_SLV4_DLY_EN    0x10
#define BITS_SLV3_DLY_EN    0x08
#define BITS_SLV2_DLY_EN    0x04
#define BITS_SLV1_DLY_EN    0x02
#define BITS_SLV0_DLY_EN    0x01

#define BIT_RAW_RDY_EN		0x01
#define BIT_INT_ANYRD_2CLEAR	0x10

#define BITS_INT_STATUS_FIFO_OVERFLOW	0x10

#define BITS_I2C_MST_CLK_400_KHZ	0x0D

#ifndef M_PI_F
#define M_PI_F 3.14159265358979323846f
#endif

#if defined(__DF_EDISON)
// update frequency 250 Hz
#define MPU9250_MEASURE_INTERVAL_US 4000
#else
// update frequency 1000 Hz
#define MPU9250_MEASURE_INTERVAL_US 1000
#endif

// -2000 to 2000 degrees/s, 16 bit signed register, deg to rad conversion
#define GYRO_RAW_TO_RAD_S 	 (2000.0f / 32768.0f * M_PI_F / 180.0f)

// TODO: include some common header file (currently in drv_sensor.h).
#define DRV_DF_DEVTYPE_MPU9250 0x41

#define MPU_WHOAMI_9250			0x71

#pragma pack(push, 1)
struct fifo_packet {
	int16_t accel_x;
	int16_t accel_y;
	int16_t accel_z;
	int16_t temp;
	int16_t gyro_x;
	int16_t gyro_y;
	int16_t gyro_z;
};
struct fifo_packet_with_mag {
	int16_t accel_x;
	int16_t accel_y;
	int16_t accel_z;
	int16_t temp;
	int16_t gyro_x;
	int16_t gyro_y;
	int16_t gyro_z;
	char mag_st1; // 14 mag ST1 (1B)
	int16_t mag_x; // 15-16 (2B)
	int16_t mag_y; // 17-18 (2B)
	int16_t mag_z; // 19-20 (2B)
	char mag_st2; // 21 mag ST2 (1B)
};
// This data structure is a copy of the segment of the above fifo_packet_with_mag data
// struture that contains mag data.
struct mag_data {
	char mag_st1; // mag ST1 (1B)
	int16_t mag_x; // uT (2B)
	int16_t mag_y; // uT (2B)
	int16_t mag_z; // uT (2B)
	char mag_st2; // mag ST2 (1B)
};
#pragma pack(pop)

class MPU9250: public ImuSensor
{
public:
	MPU9250(const char *device_path, bool mag_enabled = false) :
		ImuSensor(device_path, MPU9250_MEASURE_INTERVAL_US, mag_enabled), // true = mag is enabled
		_last_temp_c(0.0f),
		_temp_initialized(false),
		_mag_enabled(mag_enabled),
#if defined(__DF_EDISON)
		_packets_per_cycle_filtered(4.0f), // The FIFO is supposed to run at 1kHz and we sample at 250Hz.
#else
		_packets_per_cycle_filtered(8.0f), // The FIFO is supposed to run at 8kHz and we sample at 1kHz.
#endif
		_mag(nullptr)
	{
		m_id.dev_id_s.devtype = DRV_DF_DEVTYPE_MPU9250;
		// TODO: does the WHOAMI make sense as an address?
		m_id.dev_id_s.address = MPU_WHOAMI_9250;
	}

	// @return 0 on success, -errno on failure
	int writeReg(int reg, uint8_t val)
	{
		return _writeReg(reg, val);
	}

	int readReg(uint8_t address, uint8_t &val)
	{
		return _readReg(address, val);
	}

	int modifyReg(uint8_t address, uint8_t clearbits, uint8_t setbits)
	{
		return _modifyReg(address, clearbits, setbits);
	}

	// @return 0 on success, -errno on failure
	virtual int start();

	// @return 0 on success, -errno on failure
	virtual int stop();

protected:
	virtual void _measure();
	virtual int _publish(struct imu_sensor_data &data);

private:
	// @returns 0 on success, -errno on failure
	int mpu9250_init();

	// @returns 0 on success, -errno on failure
	int mpu9250_deinit();

	// @return the number of FIFO bytes to collect
	int get_fifo_count();

	void reset_fifo();

	void clear_int_status();

	float _last_temp_c;
	bool _temp_initialized;
	bool _mag_enabled;
	float _packets_per_cycle_filtered;

	MPU9250_mag *_mag;
};

}
// namespace DriverFramework

