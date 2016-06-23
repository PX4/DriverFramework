#include <stdint.h>
#include <string.h>
#include "math.h"
#include "DriverFramework.hpp"
#include "LSM9DS1.hpp"


#define G_SI 9.80665
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
		_gyro_scale = 0.00875;
		break;

	case BITS_FS_G_500DPS:
		_gyro_scale = 0.0175;
		break;

	case BITS_FS_G_2000DPS:
		_gyro_scale = 0.07;
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
		_acc_scale = 0.000061;
		break;

	case BITS_FS_XL_4G:
		_acc_scale = 0.000122;
		break;

	case BITS_FS_XL_8G:
		_acc_scale = 0.000244;
		break;

	case BITS_FS_XL_16G:
		_acc_scale = 0.000732;
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
		_mag_scale = 0.00014;
		break;

	case BITS_FS_M_8Gs:
		_mag_scale = 0.00029;
		break;

	case BITS_FS_M_12Gs:
		_mag_scale = 0.00043;
		break;

	case BITS_FS_M_16Gs:
		_mag_scale = 0.00058;
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
	m_sensor_data.fifo_overflow_counter = 0;
	m_sensor_data.fifo_corruption_counter = 0;
	m_sensor_data.gyro_range_hit_counter = 0;
	m_sensor_data.accel_range_hit_counter = 0;

	m_sensor_data.fifo_sample_interval_us = 0;
	m_sensor_data.is_last_fifo_sample = false;

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

	// Enable/clear the FIFO of any residual data and enable the I2C master clock, if the mag is
	// enabled.
	uint8_t ctrl_reg9;
	_readReg(LSM9DS1XG_CTRL_REG9, ctrl_reg9);

	_writeReg(LSM9DS1XG_CTRL_REG9, ctrl_reg9 |
		  BITS_USER_CTRL_REG9_FIFO_EN | BITS_USER_CTRL_REG9_FIFO_TEMP_EN);

	reset_fifo();

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

int LSM9DS1::get_fifo_count()
{

	// Use 1 MHz for normal registers.
	_setBusFrequency(SPI_FREQUENCY_1MHZ);

	uint8_t fifo_src;
	_readReg(LSM9DS1XG_FIFO_SRC, fifo_src);

	int fss = 0;

	if (fifo_src != 0) {
		fss = swap16(fifo_src & BITS_USER_CTRL_FIFO_SRC_FSS);

	} else {
		DF_LOG_ERR("FIFO read count failed");
	}

	return fss;
}

void LSM9DS1::reset_fifo()
{
	// Use 1 MHz for normal registers.
	_setBusFrequency(SPI_FREQUENCY_1MHZ);

	int result;

	result = _writeReg(LSM9DS1XG_FIFO_CTRL,
			   BITS_USER_CTRL_FIFO_FMODE_RST);

	if (result != 0) {
		DF_LOG_ERR("FIFO FMODE reset failed");
	}

	result = _writeReg(LSM9DS1XG_FIFO_CTRL,
			   BITS_USER_CTRL_FIFO_FMODE_EN);

	if (result != 0) {
		DF_LOG_ERR("FIFO FMODE enable failed");
	}
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

	if (int_status & BITS_INT_STATUS_FIFO_OVRN) {
		//No need to reset as it is configured to be in continuous mode.
		m_synchronize.lock();
		++m_sensor_data.fifo_overflow_counter;
		DF_LOG_ERR("FIFO overrun");
		m_synchronize.unlock();

		return;
	}

	// Get FIFO byte count to read and floor it to the report size.
	int fifo_count = get_fifo_count();

	if (fifo_count < 0) {
		m_synchronize.lock();
		++m_sensor_data.error_counter;
		m_synchronize.unlock();
		return;
	}

	const unsigned read_len = MIN((unsigned)fifo_count, LSM9DS1_MAX_FIFO_LEN);
	_setBusFrequency(SPI_FREQUENCY_10MHZ);

	for (unsigned packet_index = 0; packet_index < read_len; ++packet_index) {

		fifo_packet _report;
		fifo_packet *report = &_report;
		uint8_t response[6];
		int16_t bit_data[3];

		// Read Temperature
		_bulkRead(LSM9DS1XG_OUT_TEMP_L, &response[0], 2);
		report->temp = (((int16_t) response[1] << 8) | response[0]);

		// Read Accelerometor
		_bulkRead(LSM9DS1XG_OUT_X_L_XL, &response[0], 6);

		for (int i = 0; i < 3; i++) {
			bit_data[i] = ((int16_t)response[2 * i + 1] << 8) | response[2 * i] ;
		}

		report->accel_x = bit_data[0];
		report->accel_y = bit_data[1];
		report->accel_z = bit_data[2];

		// Read gyroscope
		_bulkRead(LSM9DS1XG_OUT_X_L_G, &response[0], 6);

		for (int i = 0; i < 3; i++) {
			bit_data[i] = ((int16_t)response[2 * i + 1] << 8) | response[2 * i] ;
		}

		report->gyro_x = bit_data[0];
		report->gyro_y = bit_data[1];
		report->gyro_z = bit_data[2];

		if (_mag_enabled) {
			_mag->bulkRead(LSM9DS1M_OUT_X_L_M, &response[0], 6);

			for (int i = 0; i < 3; i++) {
				bit_data[i] = ((int16_t)response[2 * i + 1] << 8) | response[2 * i];
			}

			report->mag_x = bit_data[0];
			report->mag_y = bit_data[1];
			report->mag_z = bit_data[2];

		} else {
			report->mag_x = 0;
			report->mag_y = 0;
			report->mag_z = 0;
		}

#if 0
		/* TODO: add ifdef for endianness */
		report->accel_x = swap16(report->accel_x);
		report->accel_y = swap16(report->accel_y);
		report->accel_z = swap16(report->accel_z);
		report->temp = swap16(report->temp);
		report->gyro_x = swap16(report->gyro_x);
		report->gyro_y = swap16(report->gyro_y);
		report->gyro_z = swap16(report->gyro_z);
#endif

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

		const float temp_c = float(report->temp) / 256.0f + 25.0f;

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
		m_sensor_data.accel_m_s2_x = double(report->accel_x)
					     * double(G_SI) * double(_acc_scale);
		m_sensor_data.accel_m_s2_y = double(report->accel_y)
					     * double(G_SI) * double(_acc_scale);
		m_sensor_data.accel_m_s2_z = double(report->accel_z)
					     * double(G_SI) * double(_acc_scale);

		m_sensor_data.temp_c = temp_c;
		m_sensor_data.gyro_rad_s_x = (PI / 180) * float(report->gyro_x) * _gyro_scale;
		m_sensor_data.gyro_rad_s_y = (PI / 180) * float(report->gyro_y) * _gyro_scale;
		m_sensor_data.gyro_rad_s_z = (PI / 180) * float(report->gyro_z) * _gyro_scale;

		m_sensor_data.mag_ga_x = 100.0 * (double(report->mag_x) * double(_mag_scale));
		m_sensor_data.mag_ga_y = 100.0 * (double(report->mag_y) * double(_mag_scale));
		m_sensor_data.mag_ga_z = 100.0 * (double(report->mag_z) * double(_mag_scale));

		// Pass on the sampling interval between FIFO samples at 8kHz.
		m_sensor_data.fifo_sample_interval_us = 125;

		// Flag if this is the last sample, and _publish() should wrap up the data it has received.
		m_sensor_data.is_last_fifo_sample = ((packet_index + 1) == (read_len));

		++m_sensor_data.read_counter;

		// Generate debug output every second, assuming that a sample is generated every
		// 125 usecs
#ifdef LSM9DS1_DEBUG

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

#ifdef LSM9DS1_DEBUG

		if (_mag_enabled && mag_error == 0) {
			if ((m_sensor_data.read_counter % 10000) == 0) {
				DF_LOG_INFO("     mag:  [%f, %f, %f] ga",
					    m_sensor_data.mag_ga_x, m_sensor_data.mag_ga_y, m_sensor_data.mag_ga_z);
			}
		}

#endif

		_publish(m_sensor_data);

		m_synchronize.signal();
		m_synchronize.unlock();
	}
}

int LSM9DS1::_publish(struct imu_sensor_data &data)
{
	// TBD
	return -1;
}
