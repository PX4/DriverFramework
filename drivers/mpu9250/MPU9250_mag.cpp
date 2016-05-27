/****************************************************************************
 *
 *   Copyright (C) 2016 James Y. Wilson. All rights reserved.
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

#include "MPU9250.hpp"
#include "MPU9250_mag.hpp"

using namespace DriverFramework;

// Uncomment to allow additional debug output to be generated.
// #define MPU9250_MAG_DEBUG 1

int MPU9250_mag::_convert_sample_rate_enum_to_hz(
		enum mag_sample_rate_e sample_rate)
{
	switch (sample_rate) {
		case MPU9250_MAG_SAMPLE_RATE_100HZ:
			return 100;
		case MPU9250_MAG_SAMPLE_RATE_8HZ:
			return 8;
		default:
			DF_LOG_ERR("Invalid mag sample rate detected.");
			return -1;
	}
}

int MPU9250_mag::_initialize(int output_data_rate_in_hz)
{
	int result;

	// Configure the IMU as an I2C master at 400 KHz.
	result = _imu.writeReg(MPUREG_USER_CTRL, BITS_USER_CTRL_I2C_MST_EN);
	if (result != 0) {
		DF_LOG_ERR("IMU I2C master enable failed.");
		return -1;
	}
	result = _imu.writeReg(MPUREG_I2C_MST_CTRL, BITS_I2C_MST_CLK_400_KHZ);
	if (result != 0) {
		DF_LOG_ERR("IMU I2C master bus config failed.");
		return -1;
	}
	usleep(1000);

	// Detect mag presence by reading whoami register
	if (detect() != 0) {
		DF_LOG_ERR("MPU9250 mag not detected.");
		return -1;
	}

	// Get mag calibraion data from Fuse ROM
	if (get_sensitivity_adjustment() != 0) {
		DF_LOG_ERR("Unable to read mag sensitivity adjustment");
		return -1;
	}

	// Power on and configure the mag to produce 16 bit data in continuous measurement mode.
	int mag_mode;
	if (_sample_rate == MPU9250_MAG_SAMPLE_RATE_100HZ) {
		mag_mode = BIT_MAG_CNTL1_MODE_CONTINUOUS_MEASURE_MODE_2;
	} else if (_sample_rate == MPU9250_MAG_SAMPLE_RATE_8HZ) {
		mag_mode = BIT_MAG_CNTL1_MODE_CONTINUOUS_MEASURE_MODE_1;
	} else {
		DF_LOG_ERR("Unable to select a valid mag mode.");
		return -1;
	}
	result = write_reg(MPU9250_MAG_REG_CNTL1, BIT_MAG_CNTL1_16_BITS | mag_mode);
	if (result != 0) {
		DF_LOG_ERR("Unable to configure the magnetometer mode.");
	}
	usleep(1000);

	// Slave 0 provides ST1, mag data, and ST2 data in a bulk transfer of
	// 8 bytes of data.  Use the address of ST1 in SLV0_REG as the beginning
	// register of the 8 byte bulk transfer.
	result = _imu.writeReg(MPUREG_I2C_SLV0_ADDR,
			MPU9250_AK8963_I2C_ADDR | MPU9250_AK8963_I2C_READ);
	if (result != 0) {
		DF_LOG_ERR("MPU9250 I2C slave 0 address configuration failed.");
		return -1;
	}
	result = _imu.writeReg(MPUREG_I2C_SLV0_REG, MPU9250_MAG_REG_ST1);
	if (result != 0) {
		DF_LOG_ERR("MPU9250 I2C slave 0 register configuration failed.");
		return -1;
	}
	result = _imu.writeReg(MPUREG_I2C_SLV0_CTRL, BITS_I2C_SLV0_EN | 0x08);
	if (result != 0) {
		DF_LOG_ERR("MPU9250 I2C slave 0 control configuration failed.");
		return -1;
	}
	usleep(1000);

	// Enable reading of the mag every n samples, dividing down from the
	// output data rate provided by the caller.
	int sample_rate_in_hz = _convert_sample_rate_enum_to_hz(_sample_rate);
	if (sample_rate_in_hz <= 0) {
		DF_LOG_ERR("Unable to convert the requested mag sample rate to Hz.");
		return -1;
	}
	uint8_t i2c_mst_delay = output_data_rate_in_hz / sample_rate_in_hz;
	result = _imu.writeReg(MPUREG_I2C_SLV4_CTRL, i2c_mst_delay);
	if (result != 0) {
		DF_LOG_ERR(
				"Unable to configure the I2C delay from the configured output data rate.");
		return -1;
	}
	usleep(1000);

	// Enable delayed I2C transfers for the mag on Slave 0 registers.
	result = _imu.writeReg(MPUREG_I2C_MST_DELAY_CTRL, BITS_SLV0_DLY_EN);
	if (result != 0) {
		DF_LOG_ERR("Unable to enable the I2C delay on slave 0.");
		return -1;
	}
	usleep(1000);

	return 0;
}

int MPU9250_mag::initialize(int output_data_rate_in_hz)
{
	// Retry up to 5 times to ensure successful initialization of the
	// sensor's internal I2C bus.
	int init_max_tries = 5;
	int ret = 0;
	int i;
	for (i = 0; i < init_max_tries; i++) {
#ifdef MPU9250_MAG_DEBUG
		DF_LOG_INFO("Calling _initialize(%d)", output_data_rate_in_hz);
#endif
		ret = _initialize(output_data_rate_in_hz);
		if (ret == 0) {
			break;
		}
		DF_LOG_ERR("mag initialization failed %d tries", i + 1);
		usleep(10000);
	}

	if (ret == 0) {
#ifdef MPU9250_MAG_DEBUG
		DF_LOG_INFO("mag initialization succ after %d retries", i);
#endif
		_mag_initialized = true;
	} else {
		DF_LOG_ERR("failed to initialize mag!");
	}

	return ret;
}

int MPU9250_mag::get_sensitivity_adjustment(void)
{
	int i;
	uint8_t asa[3];

	// First set power-down mode
	if (write_reg(MPU9250_MAG_REG_CNTL1, BIT_MAG_CNTL1_MODE_POWER_DOWN) != 0) {
		return -1;
	}
	usleep(10000);

	// Enable FUSE ROM, since the sensitivity adjustment data is stored in
	// compass registers 0x10, 0x11 and 0x12 which is only accessible in Fuse
	// access mode.
	if (write_reg(MPU9250_MAG_REG_CNTL1,
			BIT_MAG_CNTL1_16_BITS | BIT_MAG_CNTL1_FUSE_ROM_ACCESS_MODE) != 0) {
		return -1;
	}
	usleep(10000);

	// Get compass calibration register 0x10, 0x11, 0x12
	// store into context
	for (i = 0; i < 3; i++) {
		if (read_reg(MPU9250_MAG_REG_ASAX + i, &asa[i]) != 0) {
			return -1;
		}
		_mag_sens_adj[i] = (float) (((float) asa[i] - 128.0) / 256.0) + 1.0f;
	}

	// Leave in a power-down mode
	if (write_reg(MPU9250_MAG_REG_CNTL1, BIT_MAG_CNTL1_MODE_POWER_DOWN) != 0) {
		return -1;
	}
	usleep(10000);

#ifdef MPU9250_MAG_DEBUG
	DF_LOG_INFO("magnetometer sensitivity adjustment: %d %d %d",
			(int) (_mag_sens_adj[0] * 1000.0), (int) (_mag_sens_adj[1] * 1000.0), (int) (_mag_sens_adj[2] * 1000.0));
#endif
	return 0;
}

int MPU9250_mag::detect(void)
{
	uint8_t b = 0;

	// get mag version ID
	int retVal = read_reg(MPU9250_MAG_REG_WIA, &b);
	if (retVal != 0) {
		DF_LOG_ERR("error reading mag whoami reg: %d", retVal);
		return -1;
	}

	if (b != MPU9250_AKM_DEV_ID) {
		DF_LOG_ERR("wrong mag ID %u (expected %u)", b, MPU9250_AKM_DEV_ID);
		return -1;
	}

	return 0;
}

// int mpu_spi_write_reg_verified(int reg, uint8_t val, uint8_t mask)
int MPU9250_mag::write_imu_reg_verified(int reg, uint8_t val, uint8_t mask)
{
	int retVal;
	uint8_t b;
	int retry = 5;
	bool err_seen;

	while (retry) {
		err_seen = FALSE;
		--retry;
		retVal = _imu.writeReg(reg, val);
		if (retVal != 0) {
			err_seen = TRUE;
			continue;
		}
		retVal = _imu.readReg(reg, b);
		if (retVal != 0) {
			err_seen = TRUE;
			continue;
		}
		if ((b & mask) != val) {
			continue;
		} else {
#ifdef MPU9250_MAG_DEBUG
			DF_LOG_INFO("set_mag_reg_verified succ for reg %d=%d", reg, val);
#endif
			return 0;
		}
	}

	if (err_seen) {
		DF_LOG_ERR("set_mag_reg_verified failed for reg %d. Error %d.",
				reg, retVal);
	} else {
		DF_LOG_ERR("set_mag_reg_verified failed for reg %d. %d!=%d",
				reg, val, b);
	}

	return retVal;
}

int MPU9250_mag::read_reg(uint8_t reg, uint8_t *val)
{
	int retVal = 0;
	uint8_t b = 0;

	// Read operation on the mag using the slave 4 registers.
	retVal = write_imu_reg_verified(MPUREG_I2C_SLV4_ADDR,
			MPU9250_AK8963_I2C_ADDR | MPU9250_AK8963_I2C_READ, 0xff);
	if (retVal != 0) {
		return retVal;
	}

	// Set the mag register to read from.
	retVal = write_imu_reg_verified(MPUREG_I2C_SLV4_REG, reg, 0xff);
	if (retVal != 0) {
		return retVal;
	}

	// Read the existing value of the SLV4 control register.
	retVal = _imu.readReg(MPUREG_I2C_SLV4_CTRL, b);
	if (retVal != 0) {
		return retVal;
	}

	// Set the I2C_SLV4_EN bit in I2C_SL4_CTRL register without overwriting other
	// bits. Enable data transfer, a read transfer as configured above.
	b |= BITS_I2C_SLV4_EN;
	// Trigger the data transfer
	retVal = _imu.writeReg(MPUREG_I2C_SLV4_CTRL, b);
	if (retVal != 0) {
		return retVal;
	}

	// Continuously check I2C_MST_STATUS register value for the completion
	// of I2C transfer until timeout
	retVal = _imu.readReg(MPUREG_I2C_MST_STATUS, b);
	if (retVal != 0) {
		return retVal;
	}

	int loop_ctrl = 1000; // wait up to 1000 * 1 ms for completion
	while (((b & BITS_I2C_SLV4_DONE) == 0x00) && (--loop_ctrl)) {
		usleep(1000);
		retVal = _imu.readReg(MPUREG_I2C_MST_STATUS, b);
		if (retVal != 0) {
			return retVal;
		}
	}

	if (loop_ctrl == 0) {
		DF_LOG_ERR("I2C transfer timed out");
		return -1;
	}

	// Read the value received from the mag, and copy to the caller's out parameter.
	retVal = _imu.readReg(MPUREG_I2C_SLV4_DI, *val);
	if (retVal != 0) {
		return retVal;
	}

#ifdef MPU9250_MAG_DEBUG
	DF_LOG_INFO("Mag register %u read returned %u", reg, *val);
#endif

	return 0;
}

// int compass_write_register(uint8_t reg, uint8_t val)
int MPU9250_mag::write_reg(uint8_t reg, uint8_t val)
{
	int retVal = 0;
	uint8_t b = 0;

	// Configure a write operation to the mag using Slave 4.
	retVal = write_imu_reg_verified(MPUREG_I2C_SLV4_ADDR,
			MPU9250_AK8963_I2C_ADDR, 0xff);
	if (retVal != 0) {
		return retVal;
	}

	// Set the mag register address to write to using Slave 4.
	retVal = write_imu_reg_verified(MPUREG_I2C_SLV4_REG, reg, 0xff);
	if (retVal != 0) {
		return retVal;
	}

	// Set the value to write in the I2C_SLV4_DO register.
	retVal = write_imu_reg_verified(MPUREG_I2C_SLV4_DO, val, 0xff);
	if (retVal != 0) {
		return retVal;
	}

	// Read the current value of the Slave 4 control register.
	retVal = _imu.readReg(MPUREG_I2C_SLV4_CTRL, b);
	if (retVal != 0) {
		return retVal;
	}

	// Set I2C_SLV4_EN bit in I2C_SL4_CTRL register without overwriting other
	// bits.
	b |= BITS_I2C_SLV4_EN;
	// Trigger the data transfer from the byte now stored in the SLV4_DO register.
	retVal = _imu.writeReg(MPUREG_I2C_SLV4_CTRL, b);
	if (retVal != 0) {
		return retVal;
	}

	// Continuously check I2C_MST_STATUS regsiter value for the completion
	// of I2C transfer until timeout
	retVal = _imu.readReg(MPUREG_I2C_MST_STATUS, b);
	if (retVal != 0) {
		return retVal;
	}

	int loop_ctrl = 1000; // wait up to 1000 * 1ms for completion
	while (((b & BITS_I2C_SLV4_DONE) == 0x00) && (--loop_ctrl)) {
		usleep(1000);
		retVal = _imu.readReg(MPUREG_I2C_MST_STATUS, b);
		if (retVal != 0) {
			return retVal;
		}
	}

	if (loop_ctrl == 0) {
		DF_LOG_ERR("I2C transfer to mag timed out");
		return -1;
	}

#ifdef MPU9250_MAG_DEBUG
	DF_LOG_INFO("Magnetometer register %u set to %u", reg, val);
#endif

	return 0;
}

int MPU9250_mag::process(struct mag_data &data)
{
#ifdef MPU9250_MAG_DEBUG
	static int hofl_bit_counter = 0;
#endif

	// Check magnetic sensor overflow HOFL bit set.  No need to check the data ready bit, since
	// the sample rate divider should provide new samples at the correct interval.
	if (data.mag_st2 & BIT_MAG_HOFL) {
#ifdef MPU9250_MAG_DEBUG
		if ((++hofl_bit_counter % 1000) == 0) {
			DF_LOG_ERR("overflow HOFL bit set (x1000)");
		}
#endif
		return MAG_ERROR_DATA_OVERFLOW;
	}

	data.mag_x = ImuSensor::swap16(data.mag_x);
	data.mag_y = ImuSensor::swap16(data.mag_y);
	data.mag_z = ImuSensor::swap16(data.mag_z);

	// H_adj = H * ((ASA-128)*0.5/128 + 1)
	//       = H * ((ASA-128) / 256 + 1)
	// H is the raw compass reading, ((ASA-128) / 256 + 1) has been
	// computed and stored in compass_cal_f:
	// _mag_sens_adj[i] = (float) (((float) asa[i] - 128.0) / 256.0) + 1.0f;
	data.mag_x = (int16_t) ((int) data.mag_x * _mag_sens_adj[0]);
	data.mag_y = (int16_t) ((int) data.mag_y * _mag_sens_adj[1]);
	data.mag_z = (int16_t) ((int) data.mag_z * _mag_sens_adj[2]);

	// Convert the native units of the sensor (micro-Teslas) to Gauss
	data.mag_x /= 100;
	data.mag_y /= 100;
	data.mag_z /= 100;

	// Swap magnetometer x and y axis
	// Magnetometer X axis = Gyro and Accel Y axis
	// Magnetometer Y axis = Gyro and Accel X axis
	// Magnetometer Z axis = Gyro and Accel Z axis
	int16_t temp_mag_x = data.mag_x;
	data.mag_x = data.mag_y;
	data.mag_y = temp_mag_x;

	return 0;
}
