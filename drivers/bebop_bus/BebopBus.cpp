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

#include <string.h>
#include <math.h>

#include "DriverFramework.hpp"
#include "BebopBus.hpp"

#define BEBOP_REG_SET_ESC_SPEED 0x02
#define BEBOP_REG_GET_OBS 0x20
#define BEBOP_REG_START_BLDC 0x40
#define BEBOP_REG_STOP_BLDC 0x60
#define BEBOP_REG_TOGGLE_GPIO 0x4D
#define BEBOP_REG_CLEAR_ERROR 0x80
#define BEBOP_REG_PLAY_SOUND 0x82
#define BEBOP_REG_GET_INFO 0xA0

#define BEBOP_BUS_ERROR_NO 0x00
#define BEBOP_BUS_ERROR_EEPROM 0x01
#define BEBOP_BUS_ERROR_STALLED 0x02
#define BEBOP_BUS_ERROR_PROPELLER_SECURETY 0x03
#define BEBOP_BUS_ERROR_COMMUNICATION_LOST 0x04
#define BEBOP_BUS_ERROR_LOW_BATTERY 0x09
#define BEBOP_BUS_ERROR_LIPO_CELLS 0x0A
#define BEBOP_BUS_ERROR_BLDC 0x0B

// Rotation direction RLRL
//   front left: clockwise
//   front right: counterclockwise
//   back right: clockwise
//   back left: counterclockwise
#define BEBOP_BLDC_RLRL 0b00001010

// Rotation direction LRLR
//   front left: counterclockwise
//   front right: clockwise
//   back right: counterclockwise
//   back left: clockwise
#define BEBOP_BLDC_LRLR 0b00000101 // 

#define BEBOP_BLDC_RPM_MIN 3000
#define BEBOP_BLDC_RPM_MAX 12200

using namespace DriverFramework;

int BebopBus::start()
{
	int result = I2CDevObj::start();

	if (result < 0) {
		DF_LOG_ERR("DevObj start failed");
		DF_LOG_ERR("Unable to open the device path: %s", m_dev_path);
		return result;
	}

	result = _setSlaveConfig(BEBOP_BUS_SLAVE_ADDRESS,
				 BEBOP_BUS_BUS_FREQUENCY_IN_KHZ,
				 BEBOP_BUS_TRANSFER_TIMEOUT_IN_USECS);

	if (result < 0) {
		DF_LOG_ERR("Could not set slave config");
	}

	/* Try to talk to the sensor. */
	struct bebop_bus_info info;
	result = _get_info(&info);

	// TODO make some sanity checks
	//DF_LOG_INFO("Software Version: %d.%d", info.version_major, info.version_minor);
	//DF_LOG_INFO("Software Type: %d", info.type);
	//DF_LOG_INFO("Number of controlled motors: %d", info.n_motors_controlled);
	//DF_LOG_INFO("Number of flights: %d", info.n_flights);
	//DF_LOG_INFO("Last flight time: %d", info.last_flight_time);
	//DF_LOG_INFO("Total flight time: %d", info.total_flight_time);
	//DF_LOG_INFO("Last Error: %d\n", info.last_error);

	if (result < 0) {
		DF_LOG_ERR("Unable to communicate with the sensor");
		return result;
	}

	result = _clear_errors();

	if (result < 0) {
		DF_LOG_ERR("Unable to clear errors");
		return result;
	}

	result = DevObj::start();

	if (result < 0) {
		DF_LOG_ERR("DevObj start failed");
		return result;
	}

	return 0;
}

uint8_t BebopBus::_checksum(uint8_t initial, uint8_t *data, uint16_t packet_size)
{
	uint8_t checksum = initial;

	for (size_t i = 0; i < packet_size; i++) {
		checksum = checksum ^ data[i];
	}

	return checksum;
}

uint16_t BebopBus::_scale_to_rpm(float scale)
{
	float checked_scale = scale;

	if (checked_scale < 0.0f) {
		DF_LOG_ERR("BLDC scale out of range: %f < 0.0", static_cast<double>(checked_scale));
		checked_scale = 0.0f;
	}

	if (checked_scale > 1.0f) {
		DF_LOG_ERR("BLDC scale out of range: %f > 1.0", static_cast<double>(checked_scale));
		checked_scale = 1.0f;
	}

	// we assume that scale is a dimensionless desired thrust force in the range [0,1]
	// remap scale to the range [scale_min, 1], where scale_min represents the minimum
	// dimensionless force that we can output (because lowest motor speed is constrained).
	// Then calculate back from dimensionless force to desired motor speed
	// Force_norm = w^2 / w_max^2
	// Force_norm: dimensionless force in range [scale_min, 1]
	// w: desired motor speed in rpm
	// w_max: maximum motor speed
	float rpm_min_sq = (float)(BEBOP_BLDC_RPM_MIN * BEBOP_BLDC_RPM_MIN);
	float rmp_max_sq = (float)(BEBOP_BLDC_RPM_MAX * BEBOP_BLDC_RPM_MAX);
	float scale_min = rpm_min_sq / rmp_max_sq;

	// remap dimensionless force to range [scale_min, 1]
	scale = scale_min + (1.0f - scale_min) * scale;

	// use Force_norm = w^2 / w_max^2 and solve for w
	return sqrtf(scale * rmp_max_sq);
}

int BebopBus::_get_info(struct bebop_bus_info *info)
{
	memset(info, 0, sizeof(bebop_bus_info));

	int ret = _readReg(BEBOP_REG_GET_INFO, (uint8_t *)info, sizeof(bebop_bus_info));

	if (ret != 0) {
		DF_LOG_ERR("Unable to get information data");
		return -1;
	}

	// correct endians
	info->n_flights = swap16(info->n_flights);
	info->last_flight_time = swap16(info->last_flight_time);
	info->total_flight_time = swap32(info->total_flight_time);

	return 0;
}

int BebopBus::_get_observation_data(struct bebop_bus_observation *obs)
{
	memset(obs, 0, sizeof(bebop_bus_observation));

	int ret = _readReg(BEBOP_REG_GET_OBS, (uint8_t *)obs, sizeof(bebop_bus_observation));

	if (ret != 0) {
		DF_LOG_ERR("Unable to get obervation data");
		return -1;
	}

	uint8_t checksum_received = _checksum(0, (uint8_t *) obs, sizeof(bebop_bus_observation) - 1);

	if (checksum_received != obs->checksum) {
		DF_LOG_ERR("Incorrect checksum: Sent %u - Received %u", obs->checksum, checksum_received);
		return -1;
	}

	// Endian conversion and remove saturation bit
	obs->rpm_front_left = swap16(obs->rpm_front_left) & ~(1 << 15);
	obs->rpm_front_right = swap16(obs->rpm_front_right) & ~(1 << 15);
	obs->rpm_back_right = swap16(obs->rpm_back_right) & ~(1 << 15);
	obs->rpm_back_left = swap16(obs->rpm_back_left) & ~(1 << 15);
	obs->battery_voltage_mv = swap16(obs->battery_voltage_mv);

	return 0;
}

int BebopBus::_start_motors()
{

	// Select rotation direction
	uint8_t bits = BEBOP_BLDC_RLRL;;

	if (_writeReg(BEBOP_REG_START_BLDC, &bits, 1) != 0) {
		DF_LOG_ERR("Unable to start BLDCs");
		return -1;
	}

	return 0;
}

int BebopBus::_stop_motors()
{
	if (_writeReg(BEBOP_REG_STOP_BLDC, nullptr, 0) != 0) {
		DF_LOG_ERR("Unable to stop BLDCs");
		return -1;
	}

	return 0;
}

int BebopBus::_clear_errors()
{

	if (_writeReg(BEBOP_REG_CLEAR_ERROR, nullptr, 0) != 0) {
		DF_LOG_ERR("Unable to clear errors");
		return -1;
	}

	return 0;
}

int BebopBus::_play_sound(BebopBusSound sound)
{
	uint8_t bits = sound;

	if (_writeReg(BEBOP_REG_PLAY_SOUND, &bits, 1) != 0) {
		DF_LOG_ERR("Unable to play sound");
		return -1;
	}

	return 0;
}

int BebopBus::_toggle_gpio(BebopBusGPIO mode)
{
	uint8_t bits = mode;

	if (_writeReg(BEBOP_REG_TOGGLE_GPIO, &bits, 1) != 0) {
		DF_LOG_ERR("Unable to toggle gpio");
		return -1;
	}

	return 0;
}

int BebopBus::_set_esc_speed(const float speeds[4])
{
	struct bebop_bus_esc_speeds data;

	memset(&data, 0, sizeof(data));
	// Correct endians and scale to MIN-MAX rpm
	data.rpm_front_left = swap16(_scale_to_rpm(speeds[0]));
	data.rpm_front_right = swap16(_scale_to_rpm(speeds[1]));
	data.rpm_back_right = swap16(_scale_to_rpm(speeds[2]));
	data.rpm_back_left = swap16(_scale_to_rpm(speeds[3]));

	_speed_setpoint[0] = swap16(data.rpm_front_right);
	_speed_setpoint[1] = swap16(data.rpm_front_left);
	_speed_setpoint[2] = swap16(data.rpm_back_right);
	_speed_setpoint[3] = swap16(data.rpm_back_left);

	data.enable_security = 0x00;

	data.checksum = _checksum(BEBOP_REG_SET_ESC_SPEED, (uint8_t *) &data, sizeof(data) - 1);

	if (_writeReg(BEBOP_REG_SET_ESC_SPEED, (uint8_t *) &data, sizeof(data)) != 0) {
		DF_LOG_ERR("Unable to set ESC speed");
		return -1;
	}

	return 0;
}

int BebopBus::stop()
{
	int result = DevObj::stop();

	if (result != 0) {
		DF_LOG_ERR("DevObj stop failed");
		return result;
	}

	return 0;
}

void BebopBus::_measure()
{
	// Read the observation status from the Bebop
	struct bebop_bus_observation data;

	if (_get_observation_data(&data) < 0) {
		return;
	}

	// Publish the received data
	struct bebop_state_data publish_data;

	publish_data.rpm[0] = static_cast<uint16_t>(data.rpm_front_right);
	publish_data.rpm[1] = static_cast<uint16_t>(data.rpm_front_left);
	publish_data.rpm[2] = static_cast<uint16_t>(data.rpm_back_right);
	publish_data.rpm[3] = static_cast<uint16_t>(data.rpm_back_left);

	publish_data.battery_voltage_v = static_cast<float>(data.battery_voltage_mv) / 1000.0f;

	_publish(publish_data);
}

int BebopBus::_publish(struct bebop_state_data &data)
{
	// TBD
	return -1;
}

const char *BebopBus::strstatus(uint8_t status)
{
	switch (status) {
	case INIT:
		return "Initialize";

	case IDLE:
		return "Idle";

	case RAMPING:
		return "Ramping";

	case SPINNING_1:
		return "Spinning 1";

	case SPINNING_2:
		return "Spinning 2";

	case STOPPING:
		return "Stopping";

	case CRITICAL:
		return "Critical";

	default:
		return "Unknown";
	}
}

void BebopBus::_get_esc_speed_setpoint(uint16_t speeds_rpm[4])
{
	memcpy(speeds_rpm, _speed_setpoint, sizeof(_speed_setpoint));
}
