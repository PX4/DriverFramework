#include <stdint.h>
#include <string.h>
#include "math.h"
#include "DriverFramework.hpp"
#include "LSM9DS1.hpp"


#define G_SI 9.80665f
#define PI 3.14159f

#define MIN(_x, _y) (_x) > (_y) ? (_y) : (_x)

using namespace DriverFramework;

void LSM9DS1::set_gyro_scale(int scale)
{
	uint8_t reg;
	_readReg(LSM9DS1XG_CTRL_REG1_G, reg);
	reg &= BITS_FS_G_MASK;
	_writeReg(LSM9DS1XG_CTRL_REG1_G, reg | scale);

	switch (scale) {
	case BITS_FS_G_245DPS:
		_gyro_scale = PI / 180.0f * 0.00875f;
		break;

	case BITS_FS_G_500DPS:
		_gyro_scale = PI / 180.0f * 0.0175f;
		break;

	case BITS_FS_G_2000DPS:
		_gyro_scale = PI / 180.0f * 0.07f;
		break;
	}
}

void LSM9DS1::set_acc_scale(int scale)
{
	uint8_t reg;

	_readReg(LSM9DS1XG_CTRL_REG6_XL, reg);
	reg &= BITS_FS_XL_MASK;
	_writeReg(LSM9DS1XG_CTRL_REG6_XL, reg | scale);

	switch (scale) {
	case BITS_FS_XL_2G:
		_acc_scale = 0.000061f * G_SI;
		break;

	case BITS_FS_XL_4G:
		_acc_scale = 0.000122f * G_SI;
		break;

	case BITS_FS_XL_8G:
		_acc_scale = 0.000244f * G_SI;
		break;

	case BITS_FS_XL_16G:
		_acc_scale = 0.000732f * G_SI;
		break;
	}
}

void LSM9DS1::set_mag_scale(int scale)
{
	uint8_t reg;
	_mag->readReg(LSM9DS1M_CTRL_REG2_M, reg);
	reg &= BITS_FS_M_MASK;
	_mag->writeReg(LSM9DS1M_CTRL_REG2_M, reg | scale);

	switch (scale) {
	case BITS_FS_M_4Gs:
		_mag_scale = 0.00014f;
		break;

	case BITS_FS_M_8Gs:
		_mag_scale = 0.00029f;
		break;

	case BITS_FS_M_12Gs:
		_mag_scale = 0.00043f;
		break;

	case BITS_FS_M_16Gs:
		_mag_scale = 0.00058f;
		break;
	}

}


int LSM9DS1::lsm9ds1_init()
{
	// Use 1 MHz for normal registers.
	//TODO: I'm not sure if this should be different
	_setBusFrequency(SPI_FREQUENCY_1MHZ);

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
	m_sensor_data.gyro_range_hit_counter = 0;
	m_sensor_data.accel_range_hit_counter = 0;

	m_synchronize.unlock();

	// Enable Gyroscope
	_writeReg(LSM9DS1XG_CTRL_REG4, BITS_XEN_G | BITS_YEN_G | BITS_ZEN_G);

	// Configure Gyroscope
	_writeReg(LSM9DS1XG_CTRL_REG1_G, BITS_ODR_G_952HZ | BITS_FS_G_2000DPS);

	usleep(200);

	// Enable Accelerometer
	_writeReg(LSM9DS1XG_CTRL_REG5_XL, BITS_XEN_XL | BITS_YEN_XL | BITS_ZEN_XL);

	usleep(200);

	if (_mag_enabled && _mag == nullptr) {

		if ((_mag = new LSM9DS1M(_mag_device_path)) != nullptr) {

			int result;

			_mag->init();
			result = _mag->start();

			if (result != 0) {
				DF_LOG_ERR("Magnetometor initialization failed");
			}

		} else {
			DF_LOG_ERR("Allocation of magnetometor object failed");
		}

	}

	set_gyro_scale(BITS_FS_G_2000DPS);
	set_acc_scale(BITS_FS_XL_16G);
	set_mag_scale(BITS_FS_M_16Gs);
	return 0;
}

int LSM9DS1::LSM9DS1M::lsm9ds1m_init()
{
	int result = -1;

	result = writeReg(LSM9DS1M_CTRL_REG1_M, BITS_TEMP_COMP | BITS_OM_HIGH | BITS_ODR_M_80HZ);
	result = writeReg(LSM9DS1M_CTRL_REG2_M, BITS_FS_M_16Gs);
	result = writeReg(LSM9DS1M_CTRL_REG3_M, BITS_MD_CONTINUOUS);
	result = writeReg(LSM9DS1M_CTRL_REG4_M, BITS_OMZ_HIGH);
	result = writeReg(LSM9DS1M_CTRL_REG5_M, 0x00);

	usleep(200);
	return result;
}

int LSM9DS1::lsm9ds1_deinit()
{
	// Deallocate the resources for the mag driver, if enabled.
	if (_mag_enabled && _mag != nullptr) {
		_mag->stop();
		delete _mag;
		_mag = nullptr;
	}

	return 0;
}

int LSM9DS1::start()
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
	result = _readReg(LSM9DS1XG_WHO_AM_I, sensor_id);

	if (result != 0) {
		DF_LOG_ERR("Unable to communicate with the LSM9DS1 sensor");
		goto exit;
	}

	if (sensor_id != LSM9DS1_WHO_AM_I_ACC_GYRO) {
		DF_LOG_ERR("LSM9DS1 sensor WHOAMI wrong: 0x%X, should be: 0x%X",
			   sensor_id, LSM9DS1_WHO_AM_I_ACC_GYRO);
		result = -1;
		goto exit;
	}

	result = lsm9ds1_init();

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

int LSM9DS1::LSM9DS1M::start()
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
	result = _readReg(LSM9DS1M_WHO_AM_I, sensor_id);

	if (result != 0) {
		DF_LOG_ERR("Unable to communicate with the LSM9DS1M sensor");
		goto exit;
	}

	if (sensor_id != LSM9DS1_WHO_AM_I_MAG) {
		DF_LOG_ERR("LSM9DS1M sensor WHOAMI wrong: 0x%X, should be: 0x%X", sensor_id, LSM9DS1_WHO_AM_I_MAG);
		result = -1;
		goto exit;
	}

	lsm9ds1m_init();
	result = DevObj::start();

	if (result != 0) {
		DF_LOG_ERR("DevObj start failed");
		return result;
	}

exit:

	return result;

}

int LSM9DS1::stop()
{
	int result = lsm9ds1_deinit();

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

	// We need to wait so that all measure calls are finished before
	// closing the device.
	usleep(10000);

	return 0;
}

int LSM9DS1::LSM9DS1M::stop()
{
	int result = -1;

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

void LSM9DS1::_measure()
{
	// Use 1 MHz for normal registers.
	_setBusFrequency(SPI_FREQUENCY_1MHZ);

	uint8_t int_status = 0;
	int result = _readReg(LSM9DS1XG_STATUS_REG, int_status);

	if (result != 0) {
		m_synchronize.lock();
		DF_LOG_ERR("ACC_GYRO Error");
		++m_sensor_data.error_counter;
		m_synchronize.unlock();
		return;
	}

	_setBusFrequency(SPI_FREQUENCY_10MHZ);

	struct packet report;

	// Read Temperature
	_bulkRead(LSM9DS1XG_OUT_TEMP_L, (uint8_t *)&report.temp, 2);

	// Read Accelerometor
	_bulkRead(LSM9DS1XG_OUT_X_L_XL, (uint8_t *)&report.accel_x, 6);

	// Read gyroscope
	_bulkRead(LSM9DS1XG_OUT_X_L_G, (uint8_t *)&report.gyro_x, 6);

	/* TODO: add ifdef for endianness, RPi doesn't seem to need it */
	//report.accel_x = swap16(report.accel_x);
	//report.accel_y = swap16(report.accel_y);
	//report.accel_z = swap16(report.accel_z);
	//report.temp = swap16(report.temp);
	//report.gyro_x = swap16(report.gyro_x);
	//report.gyro_y = swap16(report.gyro_y);
	//report.gyro_z = swap16(report.gyro_z);

	if (_mag_enabled) {
		_mag->bulkRead(LSM9DS1M_OUT_X_L_M, (uint8_t *)&report.mag_x, 6);

		/* TODO: add ifdef for endianness */
		//report.mag_x = swap16(report.mag_x);
		//report.mag_y = swap16(report.mag_y);
		//report.mag_z = swap16(report.mag_z);

		DF_LOG_DEBUG("mag x: %d, y: %d, z: %d", report.mag_x, report.mag_y, report.mag_z);

	} else {
		report.mag_x = 0;
		report.mag_y = 0;
		report.mag_z = 0;
	}

	DF_LOG_DEBUG("accel x: %d, y: %d, z: %d", report.accel_x, report.accel_y, report.accel_z);
	DF_LOG_DEBUG("gyro x: %d, y: %d, z: %d", report.gyro_x, report.gyro_y, report.gyro_z);

	m_synchronize.lock();

	// Check if the full accel range of the accel has been used. If this occurs, it is
	// either a spike due to a crash/landing or a sign that the vibrations levels
	// measured are excessive.
	if (report.accel_x == INT16_MIN || report.accel_x == INT16_MAX ||
	    report.accel_y == INT16_MIN || report.accel_y == INT16_MAX ||
	    report.accel_z == INT16_MIN || report.accel_z == INT16_MAX) {
		++m_sensor_data.accel_range_hit_counter;
	}

	// Also check the full gyro range, however, this is very unlikely to happen.
	if (report.gyro_x == INT16_MIN || report.gyro_x == INT16_MAX ||
	    report.gyro_y == INT16_MIN || report.gyro_y == INT16_MAX ||
	    report.gyro_z == INT16_MIN || report.gyro_z == INT16_MAX) {
		++m_sensor_data.gyro_range_hit_counter;
	}

	// Inverting z to make the coordinate system right handed because the
	// sensors coordinate system is left handed according to the datasheet.

	m_sensor_data.accel_m_s2_x = report.accel_x * _acc_scale;
	m_sensor_data.accel_m_s2_y = report.accel_y * _acc_scale;
	m_sensor_data.accel_m_s2_z = -report.accel_z * _acc_scale;

	m_sensor_data.temp_c = float(report.temp) / 16.0f  + 25.0f;
	m_sensor_data.gyro_rad_s_x = float(report.gyro_x) * _gyro_scale;
	m_sensor_data.gyro_rad_s_y = float(report.gyro_y) * _gyro_scale;
	m_sensor_data.gyro_rad_s_z = -float(report.gyro_z) * _gyro_scale;

	m_sensor_data.mag_ga_x = -float(report.mag_x) * _mag_scale;
	m_sensor_data.mag_ga_y = float(report.mag_y) * _mag_scale;
	m_sensor_data.mag_ga_z = -float(report.mag_z) * _mag_scale;

	// We need to fake this at 1000 us.
	m_sensor_data.fifo_sample_interval_us = 1000;


	++m_sensor_data.read_counter;

	// Generate debug output every second, assuming that a sample is generated every
	// 1000 usecs
#ifdef LSM9DS1_DEBUG

	if (++m_sensor_data.read_counter % (1000000 / 1000) == 0) {

		DF_LOG_INFO("IMU: accel: [%f, %f, %f] m/s^2",
			    (double)m_sensor_data.accel_m_s2_x,
			    (double)m_sensor_data.accel_m_s2_y,
			    (double)m_sensor_data.accel_m_s2_z);
		DF_LOG_INFO("     gyro:  [%f, %f, %f] rad/s",
			    (double)m_sensor_data.gyro_rad_s_x,
			    (double)m_sensor_data.gyro_rad_s_y,
			    (double)m_sensor_data.gyro_rad_s_z);

		if (_mag_enabled) {
			DF_LOG_INFO("     mag:  [%f, %f, %f] ga",
				    (double)m_sensor_data.mag_ga_x,
				    (double)m_sensor_data.mag_ga_y,
				    (double)m_sensor_data.mag_ga_z);
		}

		DF_LOG_INFO("    temp:  %f C",
			    (double)m_sensor_data.temp_c);
	}

#endif

	_publish(m_sensor_data);

	m_synchronize.signal();
	m_synchronize.unlock();
}

int LSM9DS1::_publish(struct imu_sensor_data &data)
{
	// TBD
	return -1;
}
